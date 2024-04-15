#!usr/bin/env python3
import serial
import time
import json
import computer as pc

ser = serial.Serial("/dev/ttyACM0", 38400, timeout=0.01)
time.sleep(1)

distance = 0
while True:
	ser.reset_input_buffer()
	
	
	while ser.in_waiting <= 0:
		#time.sleep(0.01)
		pass
	data = ser.readline().decode('utf-8').rstrip()
	try:
	    info = json.loads(ack)
	    #print(info["distance"])
	    pc.setDataToFile(motionData.json, data["motion"])
	    pc.setDataToFile(sensorData.json, data["sensor"])
	    pc.setDataToFile(motion_sensorData.json, data["sensor"])
	except json.JSONDecodeError:
	    pass
