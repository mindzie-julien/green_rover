#!/usr/bin/env python3
import computer as pc
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.node import Node

def driving(server, target, order,emergency_stop, goal_handle, feedback_msg):
    dataTargetPoint = getDataFromFile(coordinate_area.json)
    dataCurrentPoint = getDataFromFile(positionData.json)
    targetPoint = dataTargetPoint.get(target)
    currentAngle = dataCurrentPoint.get("angle")
    motionValueCommands =  getDataFromFile(ArduinoMotionCommands.json)
    if(target == "None"):
        motionCommands["order"] = motionValueCommands.get(order)
        motionCommands["speed"] = motionValueCommands.get("SPEED_25")
        
    else:
        speed = order
        distance, targetAngle, correctionAngle = compute(targetPoint)
        motionData = pc.getDataFromFile("motionData.json")
        positionData  = pc.getDataFromFile("positionData.json")
        
        # robot rotation
        previousAngle = currentAngle 
        limitData = getDataFromFile(matchData.json)
        limitAngle = m.radians(limitData["angle_tolerance"])
        while (reachedValue("angle", currentAngle, targetAngle)):
            if(correctionAngle == 0 ):
                motionCommands["order"] = motionValueCommands.get("STOP")
                break
            if(abs(currentAngle - previousAngle) < limitAngle): ################################################ condition à modifier
            
                if(correctionAngle > 0 ):
                    motionCommands["order"] = motionValueCommands.get("RIDE_LEFT_FREE")
                    motionCommands["speed"] = motionValueCommands.get("SPEED_0")
                elif(correctionAngle < 0 ):
                    motionCommands["order"] = motionValueCommands.get("RIDE_RIGHT_FREE")
                    motionCommands["speed"] = motionValueCommands.get("SPEED_0")
                
                dataCurrentPoint = getDataFromFile(positionData.json)
                previousAngle = currentAngle
                currentAngle = dataCurrentPoint.get("angle")
                coord_x = dataCurrentPoint["coord_x"]
                coord_y = dataCurrentPoint["coord_y"]
                
                time.sleep(0.005)
                pc.updatePosition()
                
            ######################ordre de déplacement########################
            pc.writeData("motionCommands.json", motionCommands)
            coord_x = dataCurrentPoint["coord_x"]
            coord_y = dataCurrentPoint["coord_y"]
            ######################message interface########################
            server.get_logger().info('angle:'+str(m.degrees(currentAngle))+' deg')
            server.get_logger().info('coord_x: '+str(coord_x)+' mm')
            server.get_logger().info('coord_y: '+str(coord_y)+' mm')    
            feedback_msg.current_distance_completed = motionData["distanceCompleted"]
            feedback_msg.current_angle_delta = currentAngle
            feedback_msg.coord_x = positionData["coord_x"]
            feedback_msg.coord_y = positionData["coord_y"]
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.02)
            
        # robot translation
        limitDistance = m.radians(limitData["distance_tolerance"])
        distanceCompleted = 0
        previousDistance = 0
        while (reachedValue("distance", distanceCompleted, distance)):
            if(distance == 0 ):
                motionCommands["order"] = motionValueCommands.get("STOP")
                break
            if(abs(distanceCompleted - previousDistance) < limitDistance): ################################################ condition à modifier
            
                if(correctionAngle > 0 ):
                    motionCommands["order"] = motionValueCommands.get("RIDE_LEFT_FREE")
                    motionCommands["speed"] = motionValueCommands.get("SPEED_0")
                elif(correctionAngle < 0 ):
                    motionCommands["order"] = motionValueCommands.get("RIDE_RIGHT_FREE")
                    motionCommands["speed"] = motionValueCommands.get("SPEED_0")
                
                
                dataCurrentPoint = getDataFromFile(motionData.json)
                previousDistance = distanceCompleted
                distanceCompleted = dataCurrentPoint.get("distanceCompleted")
                time.sleep(0.005)
                pc.updatePosition()
                
            ######################ordre de déplacement########################
            pc.writeData("motionCommands.json", motionCommands) 
            coord_x = dataCurrentPoint["coord_x"]
            coord_y = dataCurrentPoint["coord_y"]
            ######################message interface########################
            server.get_logger().info('angle:'+str(m.degrees(currentAngle))+' deg')
            server.get_logger().info('coord_x: '+str(coord_x)+' mm')
            server.get_logger().info('coord_y: '+str(coord_y)+' mm')    
            feedback_msg.current_distance_completed = motionData["distanceCompleted"]
            feedback_msg.current_angle_delta = currentAngle
            feedback_msg.coord_x = positionData["coord_x"]
            feedback_msg.coord_y = positionData["coord_y"]
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.02)
        
        result = MotionCommands.Result()
        result.target_reached = "ok"
        result.order_reached = "ok"
        result.final_angle_delta_completed = currentAngle
        result.current_distance_completed = distanceCompleted
        #goal_handle.succeed()
        return result    
    

    
def computeMotion():
    motionData = pc.readData(motionData.json)
    motion_sensorData = pc.readData(motion_sensorData.json)
    

def sensing(server, state_sensor, goal_handle, feedback_msg):
    result = MotionCommands.Result()
    if(state_sensor == "on"):
        sensing = True
    if(state_sensor == "off"):
        sensing = False
    while (sensing):
        sensorData = pc.readData("sensorData.json")
        server.get_logger().info('angle: '+str()+' deg')
        server.get_logger().info('acceleration_x: '+str()+' uG')
        server.get_logger().info('acceleration_y: '+str()+' uG')    
        feedback_msg.sensor_data = sensorData
        goal_handle.publish_feedback(feedback_msg)
        
    
    result.state_sensor = state_sensor
    return result
    
