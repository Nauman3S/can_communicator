import can
import serial
import threading
import time
import subprocess
from crc import Calculator, Crc8

calculator = Calculator(Crc8.SAEJ1850)


global sendefreigabe
global speed
global data_display
data_display="0"
speed=0
sendefreigabe=False

# Define bit rates
ARBITRATION_BITRATE = 500000  # 500 kbps
DATA_BITRATE = 2000000        # 2 Mbps
#DATA_BITRATE = 500000         # 500 kbps

# Define CAN message IDs
MESSAGE_ID_1 = 0x102
MESSAGE_ID_2 = 0x262
MESSAGE_ID_3 = 0x300

com = serial.Serial ("/dev/ttyS0", 115200, timeout=0.1)    #Open port with baud rate

#display Send
def displaySend(stringCommand):
    com.write( bytes(stringCommand,encoding="raw_unicode_escape")+b"\xff\xff\xff")

# Data
Data1 = bytearray([0, 0, 15, 255, 15, 255, 48, 0, 79, 255, 0, 0, 7, 255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 7, 255, 7, 255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
Data2 = bytearray([0, 1 , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# CRC
global data1_counter_br_fd_2
global data1_checksum_br_fd_2
data1_counter_br_fd_2=0
data1_checksum_br_fd_2=0
msg1 = can.Message(is_extended_id=False,is_fd=True, bitrate_switch=True, arbitration_id=MESSAGE_ID_1, data=Data1, dlc=64)
msg2 = can.Message(is_extended_id=False,is_fd=True, bitrate_switch=True, arbitration_id=MESSAGE_ID_2, data=Data2, dlc=32)
msg3 = can.Message(is_extended_id=False,is_fd=True, bitrate_switch=True, arbitration_id=MESSAGE_ID_3, data=[0x55]*8)


# Define CAN mailbox message ID to listen to
MAILBOX_MESSAGE_ID = 0x28A

# Define the CAN bus interface (change this according to your setup)
can_interface = 'can0'

# Define the command
#command = f"sudo ip link set {can_interface} up type can bitrate 500000 sample-point 0.8 dbitrate 2000000 dsample-point 0.75 restart-ms 1000 fd on"


# Define a variable to store the received value
received_value = None

# CAN message sending function
global bus
bus = can.interface.Bus(channel=can_interface, receive_own_messages=False, local_loopback=True, fd=True, can_filters=None, bustype='socketcan')

# CAN message receiving function
def receive_messages():
    global bus
    global eye_stat_received_value
    global eye_stat_received_value2
    global eye_stat_addition
    global eye_stat_clean
    global dfov_received_value
    global dfov_received_value_raw
    global dms_state_received_value
    global length_recdata
    global data1_counter_br_fd_2
    global data1_checksum_br_fd_2
    global sendefreigabe
#   bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
    bus.set_filters([{"can_id": MAILBOX_MESSAGE_ID, "can_mask": 0xFFF, "extended": False}])

    message_read=bus.recv()
    length_recdata=len(message_read.data)
    listener = can.BufferedReader()
    for message_read in bus:
        if MAILBOX_MESSAGE_ID==message_read.arbitration_id:
            eye_stat_received_value = message_read.data[2]
            eye_stat_received_value &= 0B00000011
            eye_stat_received_value<<=1
            eye_stat_clean=eye_stat_received_value
            
            eye_stat_received_value2 = message_read.data[3]  # Extracting the first byte as an example
            eye_stat_received_value2 &= 0B10000000
            eye_stat_received_value2=eye_stat_received_value2 >> 7
            eye_stat_addition = eye_stat_clean+eye_stat_received_value2
            
            
            
            dfov_received_value= message_read.data[2]
            dfov_received_value_raw= dfov_received_value;
            dfov_received_value_raw &= 0B00011100;
            dfov_received_value_raw= dfov_received_value_raw>>2;
            
            dms_state_received_value= message_read.data[3]
            dms_state_received_value&=0B01110000;
            dms_state_received_value= dms_state_received_value>>4;
            
            #print("Received eye1:", eye_stat_received_value)
            #print("Received eye2:", eye_stat_received_value2)
            #print("Received eye_add:", eye_stat_addition)
            displaySend("t5.txt=\"{:2.1f}\"".format(eye_stat_addition))
            #print("Received dfov:", dfov_received_value)
            #print("Received dfov_raw:", dfov_received_value_raw) 
            displaySend("t3.txt=\"{:2.1f}\"".format(dfov_received_value_raw))           
            #print("Received dms:", dms_state_received_value)
            displaySend("t4.txt=\"{:2.1f}\"".format(dms_state_received_value)) 
            #print("CRC:", data1_checksum_br_fd_2)
            #print("counter:", data1_counter_br_fd_2)
            msg1.data[54] = data1_counter_br_fd_2
         
            
            print(sendefreigabe)

            data1_checksum_br_fd_2 = calculator.checksum(msg1.data)
            if(data1_counter_br_fd_2 == 15):
                data1_counter_br_fd_2 = 0
                msg1.data[55] = data1_checksum_br_fd_2
                task1.modify_data(msg1)
            else:
                data1_counter_br_fd_2=data1_counter_br_fd_2+1
                msg1.data[55] = data1_checksum_br_fd_2
                task1.modify_data(msg1)


task1 = bus.send_periodic(msg1, 0.01)
task1.modify_data(msg1)

if not isinstance(task1, can.ModifiableCyclicTaskABC):

    task1.stop()

    
task2 = bus.send_periodic(msg2, 0.1)
if not isinstance(task2, can.ModifiableCyclicTaskABC):
    task2.stop()

task3 = bus.send_periodic(msg3, 1)
if not isinstance(task3, can.ModifiableCyclicTaskABC):
    task3.stop()
    
# Create thread for receiving messages
threading.Thread(target=receive_messages).start()

# Main thread will keep running
while True:
    data_display=com.readline(5)
    if data_display[0:] == b"Start":
      sendefreigabe =True
                
    elif data_display[0:] ==b"Stop":
      sendefreigabe =False
    elif data_display[0:1] ==b"H":
      speed =int(data_display[1:])
                            
    com.reset_input_buffer()
    #com.reset_output_buffer()
    #print(speed)
    #time.sleep(0.1)   
    pass

