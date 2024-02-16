#!/usr/bin/env python
import time
import serial

global sendefreigabe
global speed
global data_display
data_display="0"
speed=0
sendefreigabe=False

com=serial.Serial(
        port='/dev/ttyS0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
)


#display Send
def displaySend(stringCommand):
    com.write( bytes(stringCommand,encoding="raw_unicode_escape")+b"\xff\xff\xff")

displaySend("t5.txt=\"{:2.1f}\"".format(1))

while 1:
	#displaySend("t5.txt=\"{:2.1f}\"".format(1))
	data_display=com.readline()
	if data_display == b"Start":
	    sendefreigabe =True
    
	elif data_display ==b"Stop":
	    sendefreigabe =False
	elif data_display[0:1] ==b"H":
	    speed =int(data_display[1:])
	    
	com.reset_input_buffer()
	com.reset_output_buffer()
	print(sendefreigabe)
	print(speed)

