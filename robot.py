#!/usr/bin/env python3
import time
import threading
import RPi.GPIO as GPIO
import json
import addons as bot
import computer as pc

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
        self.currentTarget = None
        self.previousTarget = None
        self.escapeRoute_A = pc.getDataFromFile("escapeRoute_A.json")
        self.escapeRoute_B = pc.getDataFromFile("escapeRoute_B.json")
        self.go = False
        self.busy = False

valueBoolean = True
robotStorageData = RobotStorageData()
emergency = False


def my_callback(channel):
    global valueBoolean
    valueBoolean = False

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
        
        
    
    def run(self, robotStorageData):

# Configurer l'interruption pour les deux bords (rising et falling)
        #endPointKey = None
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
            areaCoord = self.coordinate_area.get(area)
            speed = pc.selectSpeed(areaCoord)
            endPointKey = pc.selectLastTagRoute(area)
            path = pc.traceRoute(robotStorageData, endPointKey)
            path.append(area)
            print("target : " + str(area))
            for i in range(len(path)):
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
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(robotThread.gpio_num, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(robotThread.gpio_num_obstacle_forward, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)        
    #GPIO.add_event_detect(robotThread.gpio_num_obstacle_forward, GPIO.RISING, callback=my_callback_forward, bouncetime=500) 
    GPIO.setup(robotThread.gpio_num_obstacle_backward, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    
    robotThread.run(robotStorageData)  
    emergencyThread = EmergencyThread()
    #emergencyThread.run(robotThread)
