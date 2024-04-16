#!usr/bin/env python3
import serial
import time
import json
import computer as pc

ser = serial.Serial("/dev/ttyACM0", 38400, timeout=0.01)
time.sleep(1)


while True:
    ser.reset_input_buffer()
    data = {}
    time.sleep(0.01)
    motionCommands = pc.readData(motionCommands.json)
    if motionCommands:
        dataMotion = motionCommands
        data["motion"] = dataMotion
        
    actuatorCommands = pc.readData(actuatorCommands.json)
    if actuatorCommands:
        dataActuator = actuatorCommands
        data["actuator"] = dataActuator
	
    msg = json.dumps(data)
    ser.write(msg.encode('utf-8'))
    
