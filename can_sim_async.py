# pip3 install asyncio
# pip3 install serial_asyncio

import asyncio
import logging
import serial_asyncio
import serial
import aioserial
import threading
import time
import can
import re
from crc import Calculator, Crc8
# Configure the logging system
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

calculator = Calculator(Crc8.SAEJ1850)

class SerialThread:
    def __init__(self, app, port="/dev/ttyS0", baudrate=115200):
        self.app = app
        logging.info("Serial port: {}, Baudrate: {}".format(port, baudrate))
        self.serial_port = serial.Serial(port, baudrate=baudrate)
        self.running = False

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        logging.info("Serial thread starting...")
        buffer = b''  # Initialize an empty byte buffer
        while self.running:
            data = self.serial_port.read(self.serial_port.in_waiting or 1)
            if data:
                buffer += data  # Accumulate data into the buffer
                # If there's a pause (due to timeout), process the accumulated data
                if len(data) < self.serial_port.in_waiting or not self.serial_port.in_waiting:
                    # Here you might want to check if buffer contains a complete message
                    # For now, we'll just log and clear the buffer
                    if buffer.strip():  # Only process non-empty buffers
                        #try to extract speed
                        match = re.search(b'H(\d{1,4})', buffer)
                        if (match):
                            speed_str = match.group(1).decode('utf-8')  # Extract and decode the speed value
                            speed = int(speed_str)  # Convert to integer
                            
                            if 0 <= speed <= 511:  # Validate the speed value
                                logging.info("Valid speed value received: {}".format(speed))
                                # self.app.process_speed(speed)
                                self.app.process_serial_data(buffer)
                            else:
                                logging.info("Invalid speed value: {}".format(speed))
                            
                            buffer = b''  # Reset the buffer after processing
                        elif(b"Sta" in buffer or b"Sto" in buffer) :
                            logging.info("serial_data_buffer={}".format(buffer))
                            self.app.process_serial_data(buffer)
                            buffer = b''  # Reset the buffer after processing
                        
            else:
                # Optional: Handle case where no data is read
                pass
            time.sleep(0.01)  # Prevent CPU overutilization in the loop

    def send_data(self, data):
        self.serial_port.write(data)

    def stop(self):
        self.running = False
        self.thread.join()
        self.serial_port.close()
        logging.info("Serial thread stopped.")

class CANApplication:
    def __init__(self):
        self.sendefreigabe_event = asyncio.Event()
        self.sendefreigabe_event.clear()  # init false

        self.protocol = None  # This will be set in start_serial_communication
        logging.info("CAN bus initialization...")

        self.MAILBOX_MESSAGE_ID = 0x28A
        self.loop = asyncio.get_event_loop()
        self.calculator = Calculator(Crc8.SAEJ1850)
        
        self.display_connected = False
        self.can_bus = can.interface.Bus(channel='can0', receive_own_messages=False, local_loopback=True, fd=True, can_filters=None, bustype='socketcan')
        self.can_bus.set_filters([{"can_id": self.MAILBOX_MESSAGE_ID, "can_mask": 0x7FF, "extended": False}])  # Setting CAN filters as per old code
        self.data1_counter=0
        self.serial_thread = SerialThread(self)
        self.data1_counter_br_fd_2 = 0
        self.data1_checksum_br_fd_2 = 0
        self.speed = 0
        self.data_display = "0"
        self.protocol = None

        self.MESSAGE_ID_1 = 0x102
        self.MESSAGE_ID_2 = 0x262
        self.MESSAGE_ID_3 = 0x300

        self.message_intervals = {
            self.MESSAGE_ID_1: 0.01,  # Message ID 1: Send every 10 ms
            self.MESSAGE_ID_2: 0.1,   # Message ID 2: Send every 100 ms
            self.MESSAGE_ID_3: 1      # Message ID 3: Send every 1 second
        }
        self.loop = asyncio.get_event_loop()

    def start_serial_communication(self):
    
        self.serial_thread.start()

    def display_send(self, stringCommand):
        command = bytes(stringCommand, encoding="raw_unicode_escape") + b"\xff\xff\xff"
        if self.display_connected:
            self.serial_thread.send_data(command)
    
    def process_speed(self, speed, data):
        # Ensure 'data' bytearray is long enough and 'speed' is within the specified range
        if len(data) < 2:
            raise ValueError("Data bytearray must be at least 2 bytes long.")
        if not (0 <= speed <= 511):  # Speed value must be between 0 and 511 for 13-bit encoding
            raise ValueError("Speed value must be between 0 and 511.")

        # Encode the 13-bit speed into bytes 0 and 1
        # Byte 0 will have bits 0-4 (lower 5 bits of speed)
        # Byte 1 will start with the next bit of speed as its bit 0, followed by the next 7 bits
        # This effectively shifts the speed one bit to the right, starting from bit 0 of Byte 1
        byte0 = (speed >> 8) & 0x1F  # Extract bits 8-12 (5 bits) of speed for Byte 0
        byte1 = speed & 0xFF  # Extract bits 0-7 of speed for Byte 1

        # Update 'data' bytearray
        data[0] = (data[0] & 0xE0) | byte0  # Preserve upper 3 bits of byte 0, update lower 5 bits
        data[1] = byte1  # Byte 1 contains the lower 8 bits of speed

        return data


    def process_serial_data(self, data):
        logging.info("serial_data={}".format(data))
        if data.startswith(b"Sta"):
            
            self.sendefreigabe_event.set()  # Allow sending
            logging.info("Application state changed: sendefreigabe_event=True")
            self.display_connected = True
        elif data.startswith(b"Sto"):
            
            self.sendefreigabe_event.clear()  # stop sending
            logging.info("Application state changed: sendefreigabe_event=False")
            self.display_connected = False
        elif data[0:1] ==b"H":
            self.speed =int(data[1:])
            

    async def receive_can_messages(self):
        
        logging.info("CAN message receiver task starting...")

        while True:
            message = await self.loop.run_in_executor(None, self.can_bus.recv)
            if message:
                self.process_can_message(message)
            await asyncio.sleep(0)  # Yield control to allow other tasks to run


    def process_can_message(self, message):
        try:
            if self.MAILBOX_MESSAGE_ID==message.arbitration_id:
                data = message.data
                length_recdata = len(data)
                if length_recdata >= 3:
                    eye_stat_received_value = data[2] & 0B00000011
                    eye_stat_received_value <<= 1
                    eye_stat_received_value2 = data[3] & 0B10000000
                    eye_stat_received_value2 >>= 7
                    eye_stat_addition = eye_stat_received_value + eye_stat_received_value2
                    dfov_received_value = data[2] & 0B00011100
                    dfov_received_value >>= 2
                    dms_state_received_value = data[3] & 0B01110000
                    dms_state_received_value >>= 4
                    self.display_send(f"t5.txt=\"{eye_stat_addition}\"")
                    self.display_send(f"t3.txt=\"{dfov_received_value}\"")
                    self.display_send(f"t4.txt=\"{dms_state_received_value}\"")
        except:
            logging.error("Exception in CAN message processing task", exc_info=True)

    async def send_message_with_interval(self, message_id, interval):
        data1 = bytearray(64)
        data1 = bytearray([0, 0, 15, 255, 15, 255, 48, 0, 79, 255, 0, 0, 7, 255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 7, 255, 7, 255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        data2 = bytearray(32)
        data2 = bytearray([0, 1 , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        data3 = [0x55] * 8
        while True:
            await self.sendefreigabe_event.wait()
            if message_id==self.MESSAGE_ID_1:
                data1=self.process_speed(self.speed, data1)
                # Update counter and CRC in msg1 data before sending
                self.data1_counter = (self.data1_counter + 1) % 16
                data1[54] = self.data1_counter
                self.data1_checksum = calculator.checksum(data1[:55]) # Assume the last byte is for checksum
                data1[55] = self.data1_checksum
                msg = can.Message(arbitration_id=message_id, data=data1, is_extended_id=False, is_fd=True, bitrate_switch=True)
            elif message_id==self.MESSAGE_ID_2:
                msg = can.Message(arbitration_id=message_id, data=data2, is_extended_id=False, is_fd=True, bitrate_switch=True)
            elif message_id==self.MESSAGE_ID_3:
                msg = can.Message(arbitration_id=message_id, data=data3, is_extended_id=False, is_fd=True, bitrate_switch=True)
            
            self.can_bus.send(msg)
            await asyncio.sleep(interval)
        
                
    async def send_can_messages(self):
        logging.info("Preparing to send CAN messages")

        
        # Schedule message sending tasks
        tasks = [
            self.send_message_with_interval(self.MESSAGE_ID_1, self.message_intervals[self.MESSAGE_ID_1]),
            self.send_message_with_interval(self.MESSAGE_ID_2, self.message_intervals[self.MESSAGE_ID_2]),
            self.send_message_with_interval(self.MESSAGE_ID_3, self.message_intervals[self.MESSAGE_ID_3])
        ]
        await asyncio.gather(*tasks)
        
    
    def run(self):
        self.loop.create_task(self.send_can_messages())
        self.loop.create_task(self.receive_can_messages())

        self.loop.run_forever()
async def close_can_bus(can_bus):
    can_bus.shutdown()
    logging.info("Can_bus shutdown initiated.")

def main():
    logging.info("Application starting...")
    app = CANApplication()
    app.start_serial_communication()
    app.run()

    try:
        app.display_send('t5.txt="Hello, World!"')
        
    finally:
        # Shutdown code to safely close all resources
        if app.can_bus:
            app.can_bus.shutdown()
    
    # Keep the tasks running indefinitely
    

if __name__ == "__main__":
    
    main()
    logging.info("Application shutdown initiated.")
    

