#include <CRC.h>
#include <CRC8.h>


#include <SPI.h>
#include "mcp2518fd_can.h"


//#include <MCUFRIEND_kbv.h>
//#include <TouchScreen.h>


#define MAX_DATA_SIZE_32 32
#define MAX_DATA_SIZE_64 64
#define MAX_DATA_SIZE_3 3
#define MAX_DATA_SIZE_7 7
#define MAX_DATA_SIZE_20 20
#define MAX_DATA_SIZE 64
#define B111000001111 3599
CRC8 crc;

// pin for CAN-FD Shield
const int SPI_CS_PIN  = 10; // D10 da D9 schon durch das TFT-Touchsdisplay belegt ist, D9 auf D10 wurde Hardwareseitig geändert.
const int CAN_INT_PIN = 2;


        unsigned char eyes_status_temp=0;
        unsigned char eyes_status_addition=0;


// pin for CANBed FD
//const int SPI_CS_PIN  = 17;
//const int CAN_INT_PIN = 7;

mcp2518fd CAN(SPI_CS_PIN); // Set CS pin

unsigned char stmp[MAX_DATA_SIZE_32] = {0};
unsigned char stmp64[MAX_DATA_SIZE_64] = {0};

///// Unterschiedliche Startzeiten
unsigned long startzeit_1 = 0;
unsigned long startzeit_2 = 0;
unsigned long startzeit_3 = 0;
unsigned long startzeit_4 = 0;
unsigned long startzeit_5 = 0;
unsigned long startzeit_6 = 0;


// Definitionen der einzelnen Schaltzeiten
#define        laufzeit_1   1000UL  //1000ms
#define        laufzeit_2    245UL  //250ms
#define        laufzeit_3    495UL  //500ms
#define        laufzeit_4     95UL  //100ms
#define        laufzeit_5      5UL  //10ms
#define        laufzeit_6   2000UL  //2000ms


//unsigned char stmpf_COMMANDES_VSM200[3]       = {2, 0, 0};
unsigned char stmpf_ADAS_FD_INFO2[32]       = {0, 1 , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char stmpf_BRAKE_FD_2[64]       = {0, 0, 15, 255, 15, 255, 48, 0, 79, 255, 0, 0, 7, 255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 7, 255, 7, 255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//unsigned char stmpf_DYN_VOL_03F[7]       = {0, 0 , 0, 0, 0, 0, 0};


unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[MAX_DATA_SIZE];
unsigned char dms_dfov_area=0;
unsigned char dms_dfov_area_raw=0;
unsigned char dms_state=0;
unsigned char eye_status=0;
unsigned char eye_status_clean=0;



// COUNTER VARIABLES
int counter_brake_fd_2 = 0;

int counter_bcm_fd_10  = 0;
int counter_brake_fd_5 = 0;

int counter_bcm_fd_27    = 0;
int counter_transm_fd_4  = 0;
int counter_engine_fd_7  = 0;

int counter_orc_fd_1     = 0;
int counter_brake_fd_4   = 0;
int counter_eps_fd_2     = 0;
int counter_adas_fd_hmi  = 0;

int counter_adas_info   = 0;
int counter_bcm_fd_2  = 0;

int slidervalue_byte0 = 0;
int slidervalue_byte1 = 0;
int slidervalue_raw=0;



  
// SENDEFREIGABE
//int sendefreigabe = 1; //auf 1 um dauer zu Senden
bool sendefreigabe = false;
bool ignition = false;



/*FARBTABELLE*/
#define BLUE 0x001F
#define GREEN 0x07E0
#define CYAN 0x07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define WHITE 0xFFFF
#define BLACK 0x0000
#define DARKGREEN       0x03E0      /*   0, 128,   0 */
#define YELLOW          0xFFE0      /* 255, 255,   0 */
#define GREENYELLOW     0xAFE5      /* 173, 255,  47 */
#define GREY    0x8410


#define YP A2  // must be an analog pin, use "An" notation!
#define XM A3  // must be an analog pin, use "An" notation!
#define YM 8   // can be a digital pin
#define XP 9   // can be a digital pin

#define MINPRESSURE 100
#define MAXPRESSURE 1000



// Display Tousch boundaries
const int TS_LEFT=907,TS_RT=136,TS_TOP=942,TS_BOT=139;
//
//// For better pressure precision, we need to know the resistance
//// between X+ and X- Use any multimeter to read it
//// For the one we're using, its 300 ohms across the X plate
//TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
//TSPoint tp;
//
//
//
//
///*This line makes an object named TFT from MCUFRIEND_kbv class and provides an SPI communication between LCD and Arduino.*/
////(int CS=A3, int RS=A2, int WR=A1, int RD=A0, int RST=A4)
//MCUFRIEND_kbv tft(A3, A2, A1, A0, A4);
////MCUFRIEND_kbv tft;


void received_data (String data_from_display){
  
  if(data_from_display == "Start"){
      sendefreigabe =true;
  }  
  else if(data_from_display =="Stop"){
    sendefreigabe =false;  
  }
  else if (data_from_display.startsWith("H")) { // Check for the slider marker
    // Remove the "S" marker and get the slider value
    String sliderValueStr = data_from_display.substring(1);
    int sliderValue = sliderValueStr.toInt();

    
    
    Serial.print("Received Slider Value: ");
    Serial.println(sliderValue);
    slidervalue_raw=sliderValue;
    //Serial.println(sliderValue,BIN);
    slidervalue_byte0=sliderValue &= B111000001111  ;    
    slidervalue_byte0= sliderValue <<= 4;
    slidervalue_byte1= slidervalue_raw>>=4;
    stmpf_BRAKE_FD_2[1]=slidervalue_byte0;
    stmpf_BRAKE_FD_2[0]=slidervalue_byte1;
    // Serial.println(slidervalue_byte0);
    // Serial.println(slidervalue_byte0,BIN);
    // Serial.println(slidervalue_byte0,HEX);
    // Serial.println(slidervalue_byte1);
    // Serial.println(slidervalue_byte1,BIN);
    // Serial.println(slidervalue_byte1,HEX);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Start Setup");
  while (!Serial) {}
  CAN.setMode(CAN_NORMAL_MODE);
  // init can bus : arbitration bitrate = 500k, data bitrate = 2M
  while (0 != CAN.begin(CAN_500K_2M))
  {
    Serial.println("CAN init fail, retry...");
    delay(100);
  }
  Serial.println("CAN init ok!");
  byte mode = CAN.getMode();
  Serial.print("CAN mode = ");
  Serial.println(mode);

  ///////////Nextion Serial ///////////
  Serial1.begin(115200);
  
}

void loop()
{
  ///////// Reading Data from Nextion   ///////////////
  if(Serial1.available()){
    String data_from_nextion = "";
    delay(30);
    while(Serial1.available()){
      
      data_from_nextion += char(Serial1.read());  
    }
    Serial.print("Data from nextion is:");
    Serial.println(data_from_nextion);
    received_data(data_from_nextion);

  }

  //---------------------------------------------2.000ms------------------------------------------//
  //Without Counter
  if (millis() - startzeit_6 >= laufzeit_6)
  {
    startzeit_6 = millis();
  }

  //---------------------------------------------250ms------------------------------------------//
  if (millis() - startzeit_2 >= laufzeit_2)
  {
    startzeit_2 = millis();
  }

  //---------------------------------------------500ms------------------------------------------//
  if (millis() - startzeit_3 >= laufzeit_3)
  {
    startzeit_3 = millis();

  }
 
  //--------------------------------------------1.000ms------------------------------------------//
if (millis() - startzeit_1 >= laufzeit_1)
  {
    startzeit_1 = millis();
    
  }

//----------------------------------------------100ms------------------------------------------//
if (millis() - startzeit_4 >= laufzeit_4)
  {
    startzeit_4 = millis();
    if(sendefreigabe==1){
    //CAN.sendMsgBuf(0x200, 0, CANFD::len2dlc(MAX_DATA_SIZE_3), stmpf_COMMANDES_VSM200);
    CAN.sendMsgBuf(0x262, 0, CANFD::len2dlc(MAX_DATA_SIZE_32), stmpf_ADAS_FD_INFO2);
        

        if(dms_dfov_area_raw<10)
        {
          Serial1.print("t3.txt=\"00"+String(dms_dfov_area_raw)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }
        else if(dms_dfov_area_raw>=10 && dms_dfov_area<100)
        {
          Serial1.print("t3.txt=\"0"+String(dms_dfov_area_raw)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }
        else
        {        
          Serial1.print("t3.txt=\""+String(dms_dfov_area_raw)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }



        
        if(dms_state<10)
        {
//        tft.print("00"+ String(dms_state));
          Serial1.print("t4.txt=\"00"+String(dms_state)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }
        else if(dms_state>=10 && dms_state<100)
        {
//        tft.print("0"+ String(dms_state));
          Serial1.print("t4.txt=\"0"+String(dms_state)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }
        else
        {        
//        tft.print(String(dms_state));
          Serial1.print("t4.txt=\""+String(dms_state)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }

        if(eyes_status_addition<10)
        {
//        tft.print("00"+ String(eyes_status_addition));
          Serial1.print("t5.txt=\"00"+String(eyes_status_addition)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }
        else if(eyes_status_addition>=10 && eyes_status_addition<100)
        {
          //tft.print("0"+ String(eyes_status_addition));
          Serial1.print("t5.txt=\"0"+String(eyes_status_addition)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }
        else
        {        
//        tft.print(String(eyes_status_addition));
          Serial1.print("t5.txt=\""+String(eyes_status_addition)+"\"");
          Serial1.write(0xff);
          Serial1.write(0xff);
          Serial1.write(0xff);
        }
    }

  }


  //---------------------------------------------10ms------------------------------------------//
if (millis() - startzeit_5 >= laufzeit_5)
  {
    startzeit_5 = millis();
    stmpf_BRAKE_FD_2[54] = counter_brake_fd_2; //Set counter to Zero
    uint8_t checksum_brake_fd_2 = crc8((uint8_t *)stmpf_BRAKE_FD_2,64,0x1D);
    stmpf_BRAKE_FD_2[55] = checksum_brake_fd_2;
	  crc.reset();

// COUNTER +1 or set to Zero
    if (counter_brake_fd_2 == 15)
    {
      counter_brake_fd_2 = 0;
    }
    else
    {
      counter_brake_fd_2++;
    }
	    
    if(sendefreigabe==1){
    //CAN.sendMsgBuf(0x03F, 0, CANFD::len2dlc(MAX_DATA_SIZE_7), stmpf_DYN_VOL_03F);
    CAN.sendMsgBuf(0x102, 0, CANFD::len2dlc(MAX_DATA_SIZE_64), stmpf_BRAKE_FD_2);


        if (CAN_MSGAVAIL == CAN.checkReceive()) 
    {
        CAN.readMsgBuf(&len, buf);            // You should call readMsgBuff before getCanId
        unsigned long id = CAN.getCanId();
        
        //Serial.print("Get Data From id: ");
        //Serial.println(id);
        //Serial.print("Len = ");
        //Serial.println(len);

            // print the data
        for (int i = 0; i < len; i++) {
            //Serial.print(buf[i]); 
            //Serial.print("\t");
            if(i==0 and id==650)
            {
              //dms_dfov_area=buf[i];
              //dms_dfov_area &= B00001111  ;
            }
            if(i==2 and id==650)
            {
              eye_status=buf[i];
              eye_status &= B00000011;
              eye_status<<=1;
              eye_status_clean=eye_status;

              dms_dfov_area=buf[i];
              dms_dfov_area_raw= dms_dfov_area;
              dms_dfov_area_raw &= B00011100;
              dms_dfov_area_raw= dms_dfov_area_raw>>2;
            }
            if(i==3 and id==650)
            {
              dms_state=buf[i];
              dms_state&=B01110000;
              dms_state= dms_state >>= 4;

              eyes_status_temp=buf[i];
              eyes_status_temp &= B10000000;
              eyes_status_temp=eyes_status_temp >>= 7;
              eyes_status_addition = bitWrite(eye_status, 0, eyes_status_temp);

              //eyes_status_addition = eye_status+eyes_status_temp;
              //eyes_status_addition >> 5;
              //Serial.println("Addition:");
              //Serial.println(eyes_status_addition,BIN);
              //Serial.println(eyes_status_temp,BIN);
              //Serial.println(eye_status_clean,BIN);

            }
            if(i==4 and id==650)
            {

            }

        }


        //Serial.println("dms_dfov_area");
        //Serial.println(dms_dfov_area);
        //Serial.println(dms_dfov_area,BIN);
        //Serial.println("dms_dfov_area_raw");
        //Serial.println(dms_dfov_area_raw);
        //Serial.println(dms_dfov_area_raw,BIN);
        //Serial.print("\t");
        //Serial.print("dmsstate");
        //Serial.print(dms_state);
        //Serial.print("\t");
        //Serial.print("eyestatus");
        //Serial.print(eye_status);
        //Serial.print("\t");
        
        //Serial.println();
      } 
    } 
    stmpf_BRAKE_FD_2[54] = 0;
  }
  
}
