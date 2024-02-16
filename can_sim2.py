import asyncio
import aioserial
import can
import logging
from crc import Calculator, Crc8


data1_counter_br_fd_2=None
eye_stat_received_value=None
eye_stat_received_value2=None
eye_stat_addition=None
eye_stat_clean=None
dfov_received_value=None
dfov_received_value_raw=None
dms_state_received_value=None
length_recdata=None
data1_counter_br_fd_2=0
data1_checksum_br_fd_2=None

# Setup logging
logging.basicConfig(level=logging.INFO)

# CRC Calculator for CAN messages
calculator = Calculator(Crc8.SAEJ1850)

# Global variables
sendefreigabe = False
speed = 0
data_display = "0"
MAILBOX_MESSAGE_ID = 0x28A

# CAN setup
can_interface = 'can0'
bus = can.interface.Bus(channel=can_interface, receive_own_messages=False, local_loopback=True, fd=True, can_filters=None, bustype='socketcan')
bus.set_filters([{"can_id": MAILBOX_MESSAGE_ID, "can_mask": 0xFFF, "extended": False}])
# Serial setup
com = aioserial.AioSerial("/dev/ttyS0", 115200, timeout=0.1)

# CAN message setup
MESSAGE_ID_1 = 0x102
MESSAGE_ID_2 = 0x262
MESSAGE_ID_3 = 0x300
msg1 = can.Message(is_extended_id=False, is_fd=True, bitrate_switch=True, arbitration_id=MESSAGE_ID_1, data=[0]*64, dlc=64)
msg2 = can.Message(is_extended_id=False, is_fd=True, bitrate_switch=True, arbitration_id=MESSAGE_ID_2, data=[0]*32, dlc=32)
msg3 = can.Message(is_extended_id=False, is_fd=True, bitrate_switch=True, arbitration_id=MESSAGE_ID_3, data=[0x55]*8)

# Asynchronous display send function
async def displaySend(stringCommand):
    await com.write_async(bytes(stringCommand, encoding="raw_unicode_escape") + b"\xff\xff\xff")

# Function to execute a blocking call in a separate thread
async def run_blocking(func, *args):
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, func, *args)

# Asynchronous CAN message receiving function
async def receive_and_process_messages():
    global bus, calculator, sendefreigabe, msg1, data1_counter_br_fd_2
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
    while True:
        try:
            message = await run_blocking(bus.recv, 1.0)  # Adjust timeout as needed
            if message:
                message_read = message
                if MAILBOX_MESSAGE_ID == message_read.arbitration_id:
                    eye_stat_received_value = message_read.data[2]
                    eye_stat_received_value &= 0B00000011
                    eye_stat_received_value <<= 1
                    eye_stat_clean = eye_stat_received_value

                    eye_stat_received_value2 = message_read.data[3]
                    eye_stat_received_value2 &= 0B10000000
                    eye_stat_received_value2 >>= 7
                    eye_stat_addition = eye_stat_clean + eye_stat_received_value2

                    dfov_received_value = message_read.data[2]
                    dfov_received_value_raw = dfov_received_value
                    dfov_received_value_raw &= 0B00011100
                    dfov_received_value_raw >>= 2

                    dms_state_received_value = message_read.data[3]
                    dms_state_received_value &= 0B01110000
                    dms_state_received_value >>= 4

                    await displaySend(f"t5.txt=\"{eye_stat_addition:2.1f}\"")
                    await displaySend(f"t3.txt=\"{dfov_received_value_raw:2.1f}\"")
                    await displaySend(f"t4.txt=\"{dms_state_received_value:2.1f}\"")

                    msg1.data[54] = data1_counter_br_fd_2

                    data1_checksum_br_fd_2 = calculator.checksum(msg1.data)
                    if data1_counter_br_fd_2 == 15:
                        data1_counter_br_fd_2 = 0
                    else:
                        data1_counter_br_fd_2 += 1
                    msg1.data[55] = data1_checksum_br_fd_2
                    # await run_blocking(bus.send, msg1)

                # await displaySend(f"Message received with ID: {message.arbitration_id}")
        except:
            d=0
# Asynchronous function to modify and send CAN messages
async def modify_and_send_messages():
    while True:
        try:
            if sendefreigabe:
                
                msg1.data[0] += 1  # Modify your message as needed
                await run_blocking(bus.send, msg1)
                await asyncio.sleep(0.01)  # Adjust the sleep as needed
            else:
                await asyncio.sleep(0.5)
        except:
            d=0

# Asynchronous function for serial data handling
async def serial_data_handling():
    global sendefreigabe, speed, data_display
    while True:
        data_display = await com.read_async(5)
        if data_display.startswith(b"Start"):
            sendefreigabe = True
            logging.info(f"sendefreigabe={sendefreigabe}")
        elif data_display.startswith(b"Stop"):
            sendefreigabe = False
            logging.info(f"sendefreigabe={sendefreigabe}")
        elif data_display.startswith(b"H"):
            speed = int(data_display[1:])
            logging.info(f"speed={speed}")
        # logging.info(f"Speed: {speed}, Sendefreigabe: {sendefreigabe}")

# Main async function
async def main():
    tasks = [
        asyncio.create_task(receive_and_process_messages()),
        asyncio.create_task(modify_and_send_messages()),
        asyncio.create_task(serial_data_handling())
    ]
    await asyncio.gather(*tasks)

# Run the main async event loop
if __name__ == "__main__":
    asyncio.run(main())
