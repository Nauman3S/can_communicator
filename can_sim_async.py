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
        self.serial_port = serial.Serial(port, baudrate=baudrate)
        self.running = False

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
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
                        
                        if (b"Start" in buffer or b"Stop" in buffer or re.search(b'H\d{1,4}', buffer)):
                            logging.info(f"serial_data_buffer={buffer}")
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

class CANApplication:
    def __init__(self):
        self.protocol = None  # This will be set in start_serial_communication

        self.MAILBOX_MESSAGE_ID = 0x28A
        self.loop = asyncio.get_event_loop()
        self.calculator = Calculator(Crc8.SAEJ1850)
        self.sendefreigabe = False
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
        self.message_intervals = {
            0x102: 0.01,  # Message ID 1: Send every 10 ms
            0x262: 0.1,   # Message ID 2: Send every 100 ms
            0x300: 1      # Message ID 3: Send every 1 second
        }
        self.loop = asyncio.get_event_loop()

    def start_serial_communication(self):
    
        self.serial_thread.start()

    def display_send(self, stringCommand):
        command = bytes(stringCommand, encoding="raw_unicode_escape") + b"\xff\xff\xff"
        if self.display_connected:
            self.serial_thread.send_data(command)

    def process_serial_data(self, data):
        logging.info(f"serial_data={data}")
        if data.startswith(b"Start"):
            self.sendefreigabe = True
            self.display_connected = True
        elif data.startswith(b"Stop"):
            self.sendefreigabe = False
            self.display_connected = False
        elif data[0:1] ==b"H":
            speed =int(data[1:])

    async def receive_can_messages(self):
        

        logging.info("Starting to receive CAN messages")
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
            d=0
    async def send_message_with_interval(self, message_id, data, interval):
        while True:
            msg = can.Message(arbitration_id=message_id, data=data, is_extended_id=False, is_fd=True, bitrate_switch=True)
            self.can_bus.send(msg)
            await asyncio.sleep(interval)
            
    async def send_can_messages(self):
        logging.info("Preparing to send CAN messages")

        MESSAGE_ID_1 = 0x102
        MESSAGE_ID_2 = 0x262
        MESSAGE_ID_3 = 0x300
        data1 = bytearray(64)
        data1 = bytearray([0, 0, 15, 255, 15, 255, 48, 0, 79, 255, 0, 0, 7, 255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 7, 255, 7, 255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        data2 = bytearray(32)
        data2 = bytearray([0, 1 , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        data3 = [0x55] * 8
        msg1 = can.Message(arbitration_id=MESSAGE_ID_1, data=data1, is_extended_id=False, is_fd=True, bitrate_switch=True)
        msg2 = can.Message(arbitration_id=MESSAGE_ID_2, data=data2, is_extended_id=False, is_fd=True, bitrate_switch=True)
        msg3 = can.Message(arbitration_id=MESSAGE_ID_3, data=data3, is_extended_id=False, is_fd=True, bitrate_switch=True)

        while True:
            if self.sendefreigabe:
                # Update counter and CRC in msg1 data before sending
                self.data1_counter = (self.data1_counter + 1) % 16
                data1[54] = self.data1_counter
                self.data1_checksum = calculator.checksum(data1[:55]) # Assume the last byte is for checksum
                data1[55] = self.data1_checksum

                # Schedule message sending tasks
                tasks = [
                    self.send_message_with_interval(MESSAGE_ID_1, data1, self.message_intervals[MESSAGE_ID_1]),
                    self.send_message_with_interval(MESSAGE_ID_2, data2, self.message_intervals[MESSAGE_ID_2]),
                    self.send_message_with_interval(MESSAGE_ID_3, data3, self.message_intervals[MESSAGE_ID_3])
                ]
                await asyncio.gather(*tasks)
            else:
                await asyncio.sleep(1)  # Adjust sleep time as needed
                # logging.info(f"sendefreigabe={self.display_send}")#thread-safe print
    def run(self):
        self.loop.create_task(self.send_can_messages())
        self.loop.create_task(self.receive_can_messages())

        self.loop.run_forever()
async def close_can_bus(can_bus):
    can_bus.shutdown()
def main():
    
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
    

