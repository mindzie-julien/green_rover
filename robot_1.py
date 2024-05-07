#!/usr/bin/env python3
import time
import threading
import RPi.GPIO as GPIO
import json
import send
import subprocess
import pince_commande as pliers
import math as m
import addons as bot
import computer as pc
import cameraUpdatePosition as vision

I2C_SLAVE_ADDR_MOTION = 0x8

class RobotStorageData():
    def __init__(self):
        self.positionData = pc.getDataFromFile("positionData.json")
        self.matchData = pc.getDataFromFile("matchData.json")
        self.checkpoint = pc.getDataFromFile("checkpoint.json")
        self.checkpointRoute = pc.getDataFromFile("checkpointRoute.json")
        self.ArduinoCommands = pc.getDataFromFile("ArduinoCommands.json" )
        self.coordinate_area = pc.getDataFromFile("coordinate_area.json" )
        self.dataId = pc.getDataFromFile("dataId.json") 
        self.motionData = pc.getDataFromFile("motionData.json") 
        self.sensorData = pc.getDataFromFile("sensorData.json") 
        self.strategy = pc.getDataFromFile("strategy.json")
        self.tagRoute = pc.getDataFromFile("tagRoute.json")
        self.currentTarget = None
        self.previousTarget = None
        self.escapeRoute_A = pc.getDataFromFile("escapeRoute_A.json")
        self.escapeRoute_B = pc.getDataFromFile("escapeRoute_B.json")
        self.escapeRoute_A_ = pc.getDataFromFile("escapeRoute_A_.json")
        self.escapeRouteDirection = pc.getDataFromFile("escapeRouteDirection.json")
        self.go = False
        self.busy = False
        self.stop = False
        self.backBusy = False
        self.goHome = False
        data ={}
        data["angle"] = m.degrees(self.positionData["angle"])
        send.send_json(data, I2C_SLAVE_ADDR_MOTION)

valueBoolean = True
robotStorageData = RobotStorageData()
emergency = False

stop_flag = threading.Event()
back_flag = threading.Event()

def cameraUpdatePosition(server):
    data = pc.readData("updatePosition.json")
    while True:
        if  data:
           server.positionData["coord_x"] = data["coord_x"]
           server.positionData["coord_y"] = data["coord_y"]
           print("ok cam coord_x: " + str(data["coord_x"]))
           print("ok cam coord_y: " + str(data["coord_y"]))
           if server.go:
               server.positionData["angle"] = data["angle"]
               print("ok cam angle: " + str(data["angle"]))
        time.sleep(0.25)

def change_flag(back_flag, stop_flag):
    # This thread will change the flag after 80 seconds
    time.sleep(75)
    print("Changing flag")
    back_flag.set()
    time.sleep(15)
    stop_flag.set()

def my_callback(channel):
    global valueBoolean
    valueBoolean = False
    
def my_callback_forward_homologation(channel):
    
    global robotStorageData
    robotStorageData.busy = True
    emergency =True
    if( robotStorageData.go == True):
        ################################################
        bot.stopHomologation(robotStorageData)
        
def my_callback_forward(channel):
    
    global robotStorageData
    robotStorageData.busy = True
    emergency =True
    if( robotStorageData.go == True):
        ################################################
        bot.avoidanceDriving(robotStorageData)
    #pc.updatePosition(robotStorageData)
    
def my_callback_backward(channel):
    
    global robotStorageData
    emergency = True
    ################################################
    emergencyStop = False
    bot.driving(self, target, order, emergencyStop)
    #pc.updatePosition(robotStorageData)


class EmergencyThread(threading.Thread):
    def __init__(self) :
        threading.Thread.__init__(self)
    
    def run(self, server):
        global emergency
        global robotStorageData
        currentTarget = None
        while True:
            
            if (emergency == True):
                currentTarget = server.currentTarget
            if GPIO.input(server.gpio_num_obstacle_forward):
                bot.avoidanceDriving()
                print("danger")
                pass
            if GPIO.input(server.gpio_num_obstacle_backward):
                pass

class RobotThread(threading.Thread):
    def __init__(self ):
        threading.Thread.__init__(self)
        self.positionData = pc.getDataFromFile("positionData.json" )
        self.matchData = pc.getDataFromFile("matchData.json")
        self.checkpoint = pc.getDataFromFile("checkpoint.json")
        self.checkpointRoute = pc.getDataFromFile("checkpointRoute.json")
        self.ArduinoCommands = pc.getDataFromFile("ArduinoCommands.json" )
        self.coordinate_area = pc.getDataFromFile("coordinate_area.json" )
        self.dataId = pc.getDataFromFile("dataId.json") 
        self.motionData = pc.getDataFromFile("motionData.json") 
        self.sensorData = pc.getDataFromFile("sensorData.json") 
        self.strategy = pc.getDataFromFile("strategy.json")
        self.gpio_num = 16
        self.gpio_num_obstacle_forward = 23
        self.gpio_num_obstacle_backward = 24
        self.currentTarget = None
        self.previousTarget = None
        self.goHome = False
        
        
    
    def run(self, robotStorageData, thread_flag):

# Configurer l'interruption pour les deux bords (rising et falling)
        #endPointKey = None
        
        thread_flag.start()
        serverBusy = False
        try:
            global valueBoolean
            launchBoolean = True
            while launchBoolean:
                time.sleep(1)
                if GPIO.input(self.gpio_num):
                    valueBoolean = False
                launchBoolean = valueBoolean
        except KeyboardInterrupt:         
            GPIO.cleanup()   
        for area, action in self.strategy.items():
            data = {}
            if stop_flag.is_set():
                break
            if(area == "None"): 
                continue
            if (action == "open"):
                pliers.pince_position('o')
            if (action == "close"):
                pliers.pince_position('f')
                
            if (action == "slow"):
                data["order"] = "slow"
                data["value"] = 0
                print(data["order"] + " : " + str(data["value"]) )
                send.send_json(data, I2C_SLAVE_ADDR_MOTION)
                #time.sleep(0.1)
                
            if (action == "fast"):
                data["order"] = "fast"
                data["value"] = 0
                print(data["order"] + " : " + str(data["value"]) )
                send.send_json(data, I2C_SLAVE_ADDR_MOTION)
                #time.sleep(0.1)
                
            if back_flag.is_set():
                print("STOP")
                robotStorageData.goHome = True
                bot.avoidanceDriving(robotStorageData)
                
            areaCoord = self.coordinate_area.get(area)
            speed = 152#pc.selectSpeed(areaCoord)
            endPointKey = area#pc.selectLastTagRoute(area)
            path = pc.traceRoute(robotStorageData, endPointKey)
            
            for cle,valeur in robotStorageData.tagRoute.items():
                
                if(valeur != cle):
                    if(area == cle):
                        path.append(area)
            #path.append(area)
            print(path)
            print("target : " + str(area))
            for i in range(len(path)):
                if stop_flag.is_set():
                    data = {}
                    data["target"] = "E"
                    data["order"] = 0
                    send.send_json(data, I2C_SLAVE_ADDR_MOTION)
                    break
                if back_flag.is_set():
                    print("back home")
                    robotStorageData.goHome = True
                    bot.avoidanceDriving(robotStorageData)
                    break
                #ajouter le code pour les actions
                data = {}
                data["target"] = path[i]
                data["order"] = speed
                target = data["target"]
                robotStorageData.currentTarget = target
                self.currentTarget = target
                order = data["order"]
                print(target)
                emergencyStop = False
                
                bot.driving(robotStorageData, target, order, emergencyStop)
                self.previousTarget = target
                robotStorageData.previousTarget = target
                
                if(robotStorageData.busy):
                    
                    serverBusy = True
                    break
            if (action == "back"):
                pliers.pince_position('o')
                time.sleep(3)
                #pliers.pince_position('f')
                data["order"] = "B"
                data["value"] = 150
                delayTime = selectTime("translation", data["value"])
                print(data["order"] + " : " + str(data["value"]) )
                send.send_json(data, I2C_SLAVE_ADDR_MOTION)
                if(robotStorageData.positionData.get("angle") == 0):
                    robotStorageData.positionData["coord_y"] = robotStorageData.positionData.get("coord_y") + 150
                if((robotStorageData.positionData.get("angle") == 180) or (robotStorageData.positionData.get("angle") == 180)):
                    robotStorageData.positionData["coord_y"] = robotStorageData.positionData.get("coord_y") - 150
                time.sleep(delayTime)
                ######################code manquant pour le recul###########################################
            if(serverBusy):
                
                path = pc.traceRoute(robotStorageData, endPointKey)
                path.append(area)
                while(robotStorageData.busy):
                    pass
                serverBusy = False
                print("old target : " + str(area))
                for i in range(len(path)):
                    #ajouter le code pour les actions
                    data = {}
                    data["target"] = path[i]
                    data["order"] = speed
                    target = data["target"]
                    robotStorageData.currentTarget = target
                    self.currentTarget = target
                    order = data["order"]
                    print("new: " + target)
                    emergencyStop = False
                
                    bot.driving(robotStorageData, target, order, emergencyStop)
                    
                robotStorageData.busy = False
        #todo a verifier mais recalcule de la vitesse    
        """data = {}   
        data["target"] = path[-1]
        data["order"] = speed
        print(data["target"])
        emergencyStop = False
        bot.driving(self, target, order, emergencyStop)"""
        
if __name__ == '__main__':
    
    #global robotStorageData
    robotThread = RobotThread()
    #cameraThread = vision.cameraUpdatePosition()
    #subprocess.call(["python3", "cameraUpdatePosition.py"])
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(robotThread.gpio_num, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(robotThread.gpio_num_obstacle_forward, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)        
    #GPIO.add_event_detect(robotThread.gpio_num_obstacle_forward, GPIO.RISING, callback=my_callback_forward, bouncetime=500) 
    #GPIO.setup(robotThread.gpio_num_obstacle_backward, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    thread_flag = threading.Thread(target=change_flag, args=(back_flag, stop_flag))
    #thread_camera = threading.Thread(target=vision.cameraPosition, args=(robotStorageData,))
    #thread_camera = threading.Thread(target=cameraUpdatePosition, args=(robotStorageData,))
    
    #thread_camera.start()
    #cameraThread.run(robotStorageData)
    robotThread.run(robotStorageData, thread_flag)  
    #cameraThread.run(robotStorageData)
    #emergencyThread = EmergencyThread()
    #emergencyThread.run(robotThread)
    

