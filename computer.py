#!/usr/bin/env python3
import subprocess
import math as m
import json
import heapq
import addons
import time

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
    data = getDataFromFile("matchData.json")
    positionData = getDataFromFile("positionData.json")
    speedData = getDataFromFile("ArduinoMotionCommands.json")
    length = data["length"]
    width = data["width"]
    #################################################################################################
    ref = m.sqrt(m.pow(width,2) + m.pow(length,2))/3 ################################################
    #################################################################################################
    diff_x = targetPoint.get("coord_x") - positionData.get("coord_x")
    diff_y = targetPoint.get("coord_y") - positionData.get("coord_y")
    mes = m.sqrt(m.pow(diff_x,2) + m.pow(diff_y,2))
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

def selectTime(motionType, value):
    if(motionType == "rotation"):
        result = abs(value)*0.03
        return result
    if(motionType == "translation"):
        result = value*0.006
        return result

def reachedValue(typeName, value, target_value):
    if(typeName == "angle"):
        
        data = getDataFromFile("matchData.json")
        tolerance = m.radians(data.get("angle_tolerance"))
        
        minValue = target_value - tolerance
        maxValue = target_value + tolerance
        if((value > minValue)and(value < maxValue)):
            
            return False
        else:
            
            return True
    if(typeName == "distance"):
        data = getDataFromFile("matchData.json")
        tolerance = data.get("distance_tolerance")
        minValue = target_value - tolerance
        maxValue = target_value + tolerance
        if((value > minValue)and(value < maxValue)):
            return False
            
        else:
            
            return True
    
def reachedPosition(targetPosition):
    try:
        try:
            currentPosition = getDataFromFile("positionData.json")
        except json.decoder.JSONDecodeError:
            return False
        data = getDataFromFile("matchData.json")
        position_range = data.get("position_range")
        x = currentPosition.get("coord_x")
        y = currentPosition.get("coord_y")
        X = targetPosition.get("coord_x")
        Y = targetPosition.get("coord_y")
        diff_x = X - x
        diff_y = Y - y
        currentRange = m.sqrt(m.pow(diff_x, 2) + m.pow(diff_y, 2))
        if(currentRange > position_range):
            return False
        else:
            return True
    except OSError:
        time.sleep(0.2)
        return False

# filename nom du fichier json pour lire les données sous format json
def getDataFromFile(filename):
    try:
    # Lire des données structurées à partir d'un fichier
        with open(filename, 'r') as fichier:
            data = json.load(fichier)
        return data
    except OSError:
        time.sleep(0.05)
# filename nom du fichier json pour ecrire les données sous format json
# data données sous format json ou dictionnaire python
def setDataToFile(filename, data):
    try:
        with open( filename, 'w') as fichier:
            json.dump(data, fichier)
    except OSError:
        time.sleep(0.05)
# coord_x valeur de la coordonnée x du robot
# coord_y valeur de la coordonnée y du robot
def selectCkeckPoint(server, coord_x, coord_y):
    positionData = server.positionData
    #positionData = getDataFromFile('positionData.json')
    checkpointList = server.checkpoint
    #checkpointList = getDataFromFile('checkpoint.json')
    
    checkpointList_distance = {}
    endPoint = {}
    """checkpointList_coord_x = {}
    checkpointList_coord_y = {}"""
    for cle, coord in checkpointList.items():
        endPoint["coord_x"] = coord[0]
        endPoint["coord_y"] = coord[1]
        checkpointList_distance[cle] = gapDistance(positionData, endPoint)
        """checkpointList_coord_x[cle] = coord[0]
        checkpointList_coord_y[cle] = coord[1]"""
    keyClosePoint = min(checkpointList_distance, key=checkpointList_distance.get)
    return keyClosePoint
    """key_coord_x = min(checkpointList_coord_x, key=lambda k: abs(checkpointList_coord_x[k] - coord_x))
    key_coord_y = min(checkpointList_coord_y, key=lambda k: abs(checkpointList_coord_y[k] - coord_y))
    if(key_coord_x != key_coord_y):
        gap_x = abs(checkpointList_coord_x[key_coord_x] - coord_x)
        gap_y = abs(checkpointList_coord_y[key_coord_y] - coord_y)
        gap_x_from_key_coord_y = abs(checkpointList_coord_x[key_coord_y] - coord_x)
        gap_y_from_key_coord_x = abs(checkpointList_coord_y[key_coord_x] - coord_y)
        if(gap_y >= gap_x):
            if(gap_y_from_key_coord_x ):
            print(1)
            return key_coord_x
        else:
            if(gap_y_from_key_coord_x ):
            print(2)
            return key_coord_y
        
    else:
        print(3)
        return key_coord_x
    """

# coord_x valeur de la coordonnée x du robot
# coord_y valeur de la coordonnée y du robot
# endPointKey clé du point cible dans le dictionnaire checkpoint.json
def traceRoute(server, endPointKey):
    positionData = server.positionData
    checkpointList = server.checkpointRoute
    #positionData = getDataFromFile('positionData.json')
    #checkpointList = getDataFromFile('checkpointRoute.json')
    coord_x = positionData["coord_x"]
    coord_y = positionData["coord_y"]
    startPoint = selectCkeckPoint(server, coord_x, coord_y)
    #startPoint = checkpointList[key_coord]
    path = dijkstra(checkpointList, startPoint, endPointKey)
    print(path)
    return path
    
def traceEscapeRoute(server, endPointKey, escapeType):
    positionData = server.positionData
    
    if escapeType == 'A':
        checkpointList = server.escapeRoute_A
        print("escapeType A")
    if escapeType == 'B':
        print("escapeType B")
        checkpointList = server.escapeRoute_B
    
    #positionData = getDataFromFile('positionData.json')
    #checkpointList = getDataFromFile('checkpointRoute.json')
    coord_x = positionData["coord_x"]
    coord_y = positionData["coord_y"]
    
    startPoint = selectCkeckPoint(server, coord_x, coord_y)
    print("test")
    #startPoint = checkpointList[key_coord]
    path = dijkstra(checkpointList, startPoint, endPointKey)
    
    return path
    
#####################################################################################################
def selectLastTagRoute(area):
    lastTagRouteJson = getDataFromFile("tagRoute.json")
    lastTagRoute = lastTagRouteJson.get(area)
    return lastTagRoute
    

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

def updatePosition(server):
    subprocess.call(["python3", "get_.py"])
    distanceJson = getDataFromFile("feedback.json")
    distance = distanceJson["distance"]
    
    currentAngle = server.positionData.get("angle")
    server.positionData["coord_x"] = distance* m.cos(m.radians(currentAngle))
    server.positionData["coord_y"] = distance* m.sin(m.radians(currentAngle))
    """dataJson = getDataFromFile("motionData.json")
    current_point = getDataFromFile("positionData.json")
    current_angle = current_point.get("angle")
    end_point = {}
    currentSpeed =  dataJson.get("speed")
    timeSample =  dataJson.get("sampleTime")
    Vx = (m.cos(current_angle))*currentSpeed
    Vy = (m.sin(current_angle))*currentSpeed
    dx = Vx*timeSample 
    dy = Vy*timeSample
    end_point["coord_x"] = dx + current_point.get("coord_x")
    end_point["coord_y"] = dy + current_point.get("coord_y")
    end_point["angle"] = current_angle
    setDataToFile("positionData.json", end_point)"""
    #return end_point
    
    
    # fonction pour calculer et retourner la valeur de l'angle de correction pour obtenir l'angle cible
    # fonction pour calculer et retourner la valeur de la distance à parcourir pour arriver sur le point cible
    # valeurs retourner à partir des dictionnaires targetPoint et currentPoint
def compute(server,targetPoint):
    currentPoint =  getDataFromFile("positionData.json")
    currentAngle =  server.positionData["angle"]
    
    
    coord_x = targetPoint.get('coord_x') - server.positionData["coord_x"]
    coord_y = targetPoint.get('coord_y') - server.positionData["coord_y"] 
    targetAngle = 0
    
    
    limitData = getDataFromFile("matchData.json")
    distanceLimit = limitData["distance_tolerance"]
    angleLimit = limitData["angle_tolerance"]
    if(abs(coord_x) < distanceLimit):
        if(coord_y > 0):
            targetAngle = (m.pi)/2
            
        if(coord_y < 0):
            targetAngle = -1*(m.pi)/2
        
    elif(coord_x > 0):
        if(coord_y >= 0):
            targetAngle = m.atan(abs(coord_y/coord_x))
        if(coord_y < 0):
            targetAngle = -1 * (m.atan(abs(coord_y/coord_x)))
    elif(coord_x < 0):
        #print("neg")
        if(coord_y >= 0):
            #print("a_p: " + str(targetAngle))
            targetAngle = m.pi - (m.atan(abs(coord_y/coord_x)))
        if(coord_y < 0):
            #print("a_n: " + str(targetAngle))
            targetAngle = m.pi + (m.atan(abs(coord_y/coord_x)))
        
    correctionAngle = targetAngle - currentAngle
    if(abs(m.degrees(correctionAngle)) < angleLimit):
        correctionAngle = 0
    distance = m.sqrt(m.pow(coord_x, 2) + m.pow(coord_y, 2))
    print("distance: " + str(distance))
    print("correctionAngle: "+ str(correctionAngle))
    return distance, targetAngle, correctionAngle
   
def gapDistance(startPoint, endPoint):
    startPoint_coord_x = startPoint.get("coord_x")
    startPoint_coord_y = startPoint.get("coord_y")
    endPoint_coord_x = endPoint.get("coord_x")
    endPoint_coord_y = endPoint.get("coord_y")   
    diff_x = endPoint_coord_x - startPoint_coord_x
    diff_y = endPoint_coord_y - startPoint_coord_y
    distance = m.sqrt(m.pow(diff_x,2) + m.pow(diff_y,2))
    return distance
    

    
