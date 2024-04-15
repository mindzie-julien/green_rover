#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math as m
import json
import heapq
import serial
import addons

def obstacleAvoidance():
    pass
    
def writeData(filename, data):
    setDataToFile(filename, data)
    
def readData(filename):
    data = getDataFromFile(filename)
    # -----à vérifier------
    nullData = {}
    setDataToFile(filename, nullData)
    return data
    # ---------------------
    """sizeData = len(data)
    if(sizeData >1):
        old_data = data[0]
        new_data = data[-1]
        old_json_data = json.dumps(old_data, sort_key=True)
        new_json_data = json.dumps(new_data, sort_key=True)
        if(old_json_data != new_json_data):
           for i in range(sizeData):
               if(i != (sizeData - 1)):
                   data.pop(i)
           return data[-1]
        else
           for i in (sizeData - 1):
               if(i != (sizeData - 1)):
                   data.pop(i)
           return None
   else
       return None"""
    
def selectSpeed(targetPoint):
    data = getDataFromFile(matchData.json)
    positionData = getDataFromFile(positionData.json)
    speedData = getDataFromFile(ArduinoMotionCommands.json)
    length = data["length"]
    width = data["width"]
    ref = m.sqrt(m.pow(width,2) + m.pow(length,2))
    diff_x = targetPoint.get("x") - positionData.get("coord_x")
    diff_y = targetPoint.get("y") - positionData.get("coord_y")
    mes = m.sqrt(m.pow(,2) + m.pow(,2))
    ratio = (mes/ref)*100
    speed = speedData["SPEED_0"]
    if(ratio > 100):
        speed = speedData["SPEED_100"]
    elif(ratio > 75):
        speed = speedData["SPEED_75"]
    elif(ratio > 50):
        speed = speedData["SPEED_50"]
    elif(ratio > 25):
        speed = speedData["SPEED_25"]
    return speed

def reachedValue(typeName, value, target_value):
    if(typeName == "angle"):
        data = getDataFromFile(matchData.json)
        tolerance = data.get("angle_tolerance")
        minValue = target_value - tolerance
        maxValue = target_value + tolerance
        if((value > minValue)and(value < maxValue)):
            return true
        else:
            return false
    if(typeName == "distance"):
        data = getDataFromFile(matchData.json)
        tolerance = data.get("distance_tolerance")
        minValue = target_value - tolerance
        maxValue = target_value + tolerance
        if((value > minValue)and(value < maxValue)):
            return true
        else:
            return false
    
def reachedPosition(currentPosition, targetPosition):
    data = getDataFromFile(matchData.json)
    position_range = data.get("position_range")
    x = currentPosition.get("x")
    y = currentPosition.get("y")
    X = targetPosition.get("X")
    Y = targetPosition.get("Y")
    diff_x = X - x
    diff_y = Y - y
    currentRange = m.sqrt(m.pow(diff_x, 2) + m.pow(diff_y, 2))
    if(currentRange > position_range):
        return false
    else:
        return true

# filename nom du fichier json pour lire les données sous format json
def getDataFromFile(filename):
    # Lire des données structurées à partir d'un fichier
    with open(filename, 'r') as fichier:
        data = json.load(fichier)
    return data

# filename nom du fichier json pour ecrire les données sous format json
# data données sous format json ou dictionnaire python
def setDataToFile(filename, data):
    with open( filename, 'w') as fichier:
        json.dump(data, fichier)

# coord_x valeur de la coordonnée x du robot
# coord_y valeur de la coordonnée y du robot
def selectCkeckPoint(coord_x, coord_y):
    ckeckpointList = getDataFromFile('checkpoint.json')
    ckeckpointList_coord_x = {}
    ckeckpointList_coord_y = {}
    for cle, coord in ckeckpointList.items():
        ckeckpointList_coord_x[cle] = coord[0]
        ckeckpointList_coord_y[cle] = coord[1]
    
    key_coord_x = min(ckeckpointList_coord_x, key=lambda k: abs(ckeckpointList_coord_x[k] - coord_x))
    key_coord_y = min(ckeckpointList_coord_y, key=lambda k: abs(ckeckpointList_coord_y[k] - coord_y))
    if(key_coord_x != key_coord_y):
        gap_x = abs(ckeckpointList_coord_x[key_coord_x] - coord_x)
        gap_y = abs(ckeckpointList_coord_y[key_coord_y] - coord_y)
        if(gap_y >= gap_x):
            return key_coord_x
        else:
            return key_coord_y
        
    else:
        return key_coord_x
    pass

# coord_x valeur de la coordonnée x du robot
# coord_y valeur de la coordonnée y du robot
# endPointKey clé du point cible dans le dictionnaire checkpoint.json
def traceRoute(coord_x, coord_y, endPointKey):
    checkpointList = getDataFromFile('checkpointRoute.json')
    startPoint = selectCkeckPoint(coord_x, coord_y)
    #startPoint = checkpointList[key_coord]
    path = dijkstra(checkpointList, startPoint, endPointKey)
    return path

# graph dictionnaire des points et chemin du plan
# start clé du point de départ dans le dictionnaire checkpoint.json de départ dans le repère
# end clé du point de fin dans le dictionnaire checkpoint.json de départ dans le repère
def dijkstra(graph, start, end):
    queue = [(0, start)]
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    path = {}

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor, distance in graph[current_node].items():
            new_distance = current_distance + distance

            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                path[neighbor] = current_node
                heapq.heappush(queue, (new_distance, neighbor))

    shortest_path = []
    while end:
        shortest_path.append(end)
        end = path.get(end)

    return shortest_path[::-1]

def updatePosition(current_point, current_angle):
    dataJson = getDataFromFile("motionData.json")
    end_point = {}
    currentSpeed =  dataJson.get("speed")
    timeSample =  dataJson.get("sampleTime")
    Vx = (m.cos(current_angle))*currentSpeed
    Vy = (m.sin(current_angle))*currentSpeed
    dx = Vx*timeSample 
    dy = Vy*timeSample
    end_point["x"] = dx + current_point.get("x")
    end_point["y"] = dy + current_point.get("y")
    return end_point
    
    
    # fonction pour calculer et retourner la valeur de l'angle de correction pour obtenir l'angle cible
    # fonction pour calculer et retourner la valeur de la distance à parcourir pour arriver sur le point cible
    # valeurs retourner à partir des dictionnaires targetPoint et currentPoint
def compute(self, targetPoint, currentPoint, currentAngle):
    coord_x = targetPoint.get('x') - currentPoint.get('x')
    coord_y = targetPoint.get('y') - currentPoint.get('y') 
    limitData = getDataFromFile(matchData.json)
    distanceLimit = limitData["distance_tolerance"]
    angleLimit = limitData["angle_tolerance"]
    if(abs(coord_x) < distanceLimit):
        if(coord_y > 0):
            targetAngle = (m.pi)/2
        if(coord_y < 0):
            targetAngle = -1*(m.pi)/2
        pass
    elif(coord_x > 0):
        if(coord_y > 0):
            targetAngle = m.atan(abs(coord_x/coord_y))
        if(coord_y < 0):
            targetAngle = -1 * (m.atan(abs(coord_x/coord_y)))
    elif(coord_x < 0):
        if(coord_y > 0):
            targetAngle = m.atan(abs(coord_x/coord_y)) + (m.pi)/2
        if(coord_y < 0):
            targetAngle = -1 * (m.atan(abs(coord_x/coord_y))) - (m.pi)/2
    correctionAngle = targetAngle - currentAngle
    if(abs(m.degrees(correctionAngle)) < angleLimit):
        correctionAngle = 0
    distance = m.sqrt(m.pow(coord_x, 2) + m.pow(coord_y, 2))
    return distance, targetAngle, correctionAngle
   
   
class ArduinoCom():
    
    def __init__(self, numberCom):
        self.numberCom = "ttyAMC%d" %numberCom
        self.baudrate = 19200
        self.serial = serial.Serial(self.numberCom, self.baudrate, timeout=1)
    def setMotionDataJson():
        pass
    """methode pour récupérer les données du capteurs"""
    def getSensorDataJson(): 
        return {}
    """methode pour récupérer le déplacement du robot"""
    def getMotionDataJson():
        return {}
