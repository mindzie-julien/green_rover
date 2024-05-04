#!/usr/bin/env python3
import computer as pc
import time
import queue
import json
import send
#import numpy as np
#import cv2
import math as m

from smbus2 import SMBus, i2c_msg



I2C_SLAVE_ADDR_SENSOR = 0x9
I2C_SLAVE_ADDR_ACTUATOR = 0x7
I2C_SLAVE_ADDR_MOTION = 0x8
ASK_FOR_LENGTH = 0x0
ASK_FOR_DATA = 0x1
I2C_LENGTH_LIMIT = 64
SLEEP_TIME = 0

def selectAvoidanceTarget(target):
    if (target == "U_R"):
        return "M_R"
    elif (target == "M_R"):
        return "D_R"
    elif (target == "D_R"):
        return "D_R_"
    elif (target == "D_R_"):
        return "D_M"
    elif (target == "D_M"):
        return "D_L_"
    elif (target == "D_L_"):
        return "D_L"
    elif (target == "D_L"):
        return "M_L"
    elif (target == "U_L"):
        return "U_L_"
    elif (target == "U_L_"):
        return "U_M"
    elif (target == "U_M"):
        return "U_R_"
    elif (target == "U_R_"):
        return "U_R"
    else :
        return "M_M"

def avoidanceDriving(server):
    
    server.busy = True
    data = {}
    data["order"] = "E"
    data["value"] = 0
    print(data["order"] + " : " + str(data["value"]) )
    
    send.send_json(data, I2C_SLAVE_ADDR_MOTION)
    time.sleep(0.1)
    target = server.previousTarget
    order = 1
    speed = order
    if (server.currentTarget != "M_M"):
       endPointKey = "M_M"
       escapeType = "A"
       #print("echoA")
       path = pc.traceEscapeRoute(server, endPointKey, escapeType)
       
       for i in range(len(path)):
           #ajouter le code pour les actions
           print("tag: " + str(i) + str(path[i]) )
           data = {}
           data["target"] = path[i]
           data["order"] = speed
           target = data["target"]
           server.currentTarget = target
           driving(server, target, order, False)
           server.previousTarget = target 
           print("ok")
           
    else:
       endPointKey = selectAvoidanceTarget(target)#"M_M"
       escapeType = "B"
       #print("echoB")
       path = pc.traceEscapeRoute(server, endPointKey, escapeType)
       for i in range(len(path)):
           #ajouter le code pour les actions
           print("tag: " + str(i) + str(path[i]) )
           data = {}
           data["target"] = path[i]
           data["order"] = speed
           target = data["target"]
           server.currentTarget = target
           driving(server, target, order, False)
           server.previousTarget = target 
           print("ok")
    #########################
    server.busy = False
    
def driving(server, target, order,emergency_stop):
    dataTargetPoint = server.coordinate_area#dataTargetPoint = pc.getDataFromFile("coordinate_area.json")
    #dataCurrentPoint = pc.getDataFromFile("positionData.json")
    targetPoint = dataTargetPoint.get(target)
    #currentAngle = dataCurrentPoint.get("angle")
    #motionValueCommands =  pc.getDataFromFile("ArduinoMotionCommands.json")
    #motionCommands = {}
    value = 0
    if(emergency_stop == True):
        pass
    else:
        server.go = False
        speed = order
        distance, targetAngle, correctionAngle = pc.compute(server,targetPoint)
        motionData = server.motionData
        positionData  = server.motionData
        #motionData = pc.getDataFromFile("motionData.json")
        #positionData  = pc.getDataFromFile("positionData.json")
        
        # robot rotation
        #previousAngle = currentAngle 
        limitData = server.matchData
        #limitData = pc.getDataFromFile("matchData.json")
        limitAngle = m.radians(limitData["angle_tolerance"])
        
        #while (pc.reachedValue("angle", currentAngle, targetAngle)):
            
        if(correctionAngle == 0 ):
            #motionCommands["order"] = motionValueCommands.get("STOP")
            #break
            pass
        #if(abs(currentAngle - previousAngle) < limitAngle): ################################################ condition à modifier
        
        if(correctionAngle > 0 ):
            #motionCommands["order"] = motionValueCommands.get("RIDE_LEFT_FREE")
            #motionCommands["speed"] = motionValueCommands.get("SPEED_0")
            data = {}
            data["order"] = "L"
            value = int(m.degrees(correctionAngle))
            data["value"] = abs(value)
            print(data["order"] + " : " + str(data["value"]) )
            send.send_json(data, I2C_SLAVE_ADDR_MOTION)
        elif(correctionAngle < 0 ):
            #motionCommands["order"] = motionValueCommands.get("RIDE_RIGHT_FREE")
            #motionCommands["speed"] = motionValueCommands.get("SPEED_0")
            data = {}
            data["order"] = "R"
            value = int(m.degrees(correctionAngle))
            data["value"] = abs(value) 
            send.send_json(data, I2C_SLAVE_ADDR_MOTION)
            print(data["order"] + " : " + str(data["value"]) )
            
        timeDelay = pc.selectTime("rotation", value)
        print("rotation timeStamp : " + str(timeDelay))
        time.sleep(timeDelay)
        server.go = True
        # robot translation
        #limitDistance = m.radians(limitData["distance_tolerance"])
        #distanceCompleted = 0
        #previousDistance = 0
        #while (pc.reachedValue("distance", distanceCompleted, distance)):
           
        if(distance != 0 ):
            
            data = {}
            
            data["order"] = "F"
            value = int(distance)
            data["value"] = value
            
            send.send_json(data, I2C_SLAVE_ADDR_MOTION)
            print("test: " + str(data["value"]))
            time.sleep(0)
                #pc.updatePosition()
            print(data["order"] + " : " + str(data["value"]) )  
              
            timeDelay = pc.selectTime("translation", value)
            print("translation timeStamp : " + str(timeDelay))
            
            time.sleep(timeDelay)
        #goal_handle.succeed()
    server.positionData["coord_x"] = server.coordinate_area[target].get("coord_x")
    server.positionData["coord_y"] = server.coordinate_area[target].get("coord_y")
    server.positionData["angle"] = targetAngle
    print(server.positionData["coord_x"] )
    print(server.positionData["coord_y"] ) 
    
    
    time.sleep(0.005)
              
    return True    
      

def sensing(server, state_sensor, goal_handle, feedback_msg):
    result = MotionCommands.Result()
    if(state_sensor == True):
        sensing = True
    if(state_sensor == False):
        sensing = False
    while (sensing):
        sensorDataJson = pc.readData("sensorData.json")
        if sensorDataJson:
            server.get_logger().info('angle: '+str(sensorDataJson["currentAngle"])+' deg')
            server.get_logger().info('acceleration_x: '+str(sensorDataJson["acceleration_x"])+' uG')
            server.get_logger().info('acceleration_y: '+str(sensorDataJson["acceleration_y"])+' uG')    
            #sensorData = json.loads(sensorDataJson)
            feedback_msg.sensor_data = str(sensorDataJson)
            
            goal_handle.publish_feedback(feedback_msg)
            
        time.sleep(0.01)
    time.sleep(1)
    result.state_sensor = state_sensor
    return result
    
"""class CameraThread(threading.Thread):
    def __init__(self) :
        threading.Thread.__init__(self)

    def run(self, server):
        while True:
            time.sleep(2)"""
            
