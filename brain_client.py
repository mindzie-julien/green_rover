#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import action_class as actions
import time
import threading
import queue

def main(args=None):
    
    
    pass

class MotionThreadClient(threading.Thread):
    def __init__(self, brainActionClient, dataInput, dataOutput):
        threading.Thread.__init__(self)
        self.brainActionClient = brainActionClient
        self.dataInput = dataInput
        self.dataOutput = dataOutput
    
    def run(self):
        while True:
            self.brainActionClient.send_motion_goal(target, order, emergency_stop)
            rclpy.spin(brain_action_client)
       
        #print("motion")
        
class SensorThreadClient(threading.Thread):
    def __init__(self, brainActionClient, dataOutput):
        threading.Thread.__init__(self)
        self.brainActionClient = brainActionClient
        self.dataOutput = dataOutput
        
    def run(self):
        while True:
            self.brainActionClient.send_sensor_goal(state_sensor)
            rclpy.spin(brain_action_client)
       
        #print("sensor")

class ActuatorThreadClient(threading.Thread):
    def __init__(self, brainActionClient, dataInput):
        threading.Thread.__init__(self)
        self.brainActionClient = brainActionClient
        self.dataInput = dataInput
        
    def run(self):
        while True:
            self.brainActionClient.send_actuator_goal(actuactor_data)
            rclpy.spin(brain_action_client)
       
        #print("actuator")

if __name__ == '__main__':
    rclpy.init()
    
    brain_action_client = actions.BrainActionClient()
    brain_action_client.get_logger().info('brain_client online')
    
    input_motion_queue = queue.Queue()
    input_actuator_queue = queue.Queue()
    output_motion_queue = queue.Queue()
    output_sensor_queue = queue.Queue()
    
    
    motionThreadClient = MotionThreadClient(brain_action_client, input_motion_queue, output_motion_queue)
    sensorThreadClient = SensorThreadClient(brain_action_client, output_sensor_queue)
    actuatorThreadClient = ActuatorThreadClient(brain_action_client, input_actuator_queue)
    motionThreadClient.start()
    sensorThreadClient.start()
    actuatorThreadClient.start()
    
    
    main()
    time.sleep(1)
    motionThreadClient.join()
    sensorThreadClient.join()
    actuatorThreadClient.join()
