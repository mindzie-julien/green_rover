#!/usr/bin/env python3
import addons as robot
import computer as master
from computer import ArduinoCom
from computer import setDataToFile
import time
import math as m
import rclpy
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.node import Node

from data_flow.action import ActuatorCommands
from data_flow.action import MotionCommands
from data_flow.action import SensorFeedback



class MotionActionServer(Node):

    def __init__(self):
        super().__init__('motion_action_server')
        self._action_server = ActionServer(
            self,
            MotionCommands,
            'MotionAction',
            self.execute_callback)


    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        #master.obstacleAvoidance()
        target = goal_handle.request.target
        order = goal_handle.request.order
        emergencyStop = goal_handle.request.emergency_stop
        result = MotionCommands.Result()
        result = robot.driving(self, target, order,emergencyStop, goal_handle, feedback_msg)
        feedback_msg = MotionCommands.Feedback()
        #feedback_msg.current_distance_completed = com.getDistance()

        '''
        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        '''
        
        #goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        #time.sleep(0.02)
        
        #result = MotionCommands.Result()
        # result.sequence = feedback_msg.partial_sequence
        return result
    
class ActuatorActionServer(Node):

    def __init__(self):
        super().__init__('actuactor_action_server')
        self._action_server = ActionServer(
            self,
            ActuatorCommands,
            'ActuatorAction',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = ActuatorCommands.Feedback()
        #feedback_msg.partial_sequence = [0, 1]

        '''
        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        '''
        goal_handle.succeed()

        result = ActuatorCommands.Result()
        # result.sequence = feedback_msg.partial_sequence
        return result

class SensorActionServer(Node):

    def __init__(self):
        super().__init__('actuator_action_server')
        self._action_server = ActionServer(
            self,
            SensorFeedback,
            'SensorAction',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = SensorFeedback.Result()
        state_sensor = goal_handle.request.state_sensor
        feedback_msg = SensorFeedback.Feedback()
        result = sensing(server, state_sensor, goal_handle, feedback_msg)
        
        goal_handle.succeed()
        
        return result

class BrainActionClient(Node):

    def __init__(self):
        super().__init__('brain_action_client')
        self.motion_client = ActionClient(self, MotionCommands, 'MotionAction')
        self.sensor_client = ActionClient(self, SensorFeedback, 'SensorAction')
        self.actuator_client = ActionClient(self, ActuatorCommands, 'ActuatorAction')
##########################################################################################################
################################        SEND GOAL          ###############################################
##########################################################################################################
    def send_motion_goal(self, target, order, emergency_stop):
        goal_msg = MotionCommands.Goal()
        goal_msg.target = target
        goal_msg.order = order
        goal_msg.emergency_stop = emergency_stop
        
        self.motion_client.wait_for_server()
    
        self.send_motion_goal_future = self.motion_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_motion)

        self.send_motion_goal_future.add_done_callback(self.goal_response_callback_motion)
        
        #return self._action_client.send_goal_async(goal_msg)
    
    def send_sensor_goal(self, state_sensor):
        goal_msg = SensorFeedback.Goal()
        goal_msg.state_sensor = state_sensor
        
        self.sensor_client.wait_for_server()

        self.send_sensor_goal_future = self.sensor_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_sensor)

        self.send_sensor_goal_future.add_done_callback(self.goal_response_callback_sensor)

        #return self._action_client.send_goal_async(goal_msg)
        
    def send_actuator_goal(self, actuactor_data):
        goal_msg = ActuatorCommands.Goal()
        goal_msg.actuator_data = actuator_data

        self.actuator_client.wait_for_server()

        self.send_actuator_goal_future = self.actuator_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_actuator)

        self.send_actuator_goal_future.add_done_callback(self.goal_response_callback_actuator)

        #return self._action_client.send_goal_async(goal_msg)
##########################################################################################################
###########################     RESPONSE    ##############################################################
##########################################################################################################
    def goal_response_callback_motion(self, future):
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_motion)
        
    def goal_response_callback_sensor(self, future):
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_sensor)
        
    def goal_response_callback_actuator(self, future):
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_actuator)
##########################################################################################################
########################               RESULT                   ##########################################
##########################################################################################################
    def get_result_callback_motion(self, future):
        result = future.result().result
        #self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def get_result_callback_sensor(self, future):
        result = future.result().result
        #self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def get_result_callback_actuator(self, future):
        result = future.result().result
        #self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()
##########################################################################################################
#################################        FEEDBACK          ###############################################
##########################################################################################################
    def feedback_callback_motion(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
        
    def feedback_callback_sensor(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
        
    def feedback_callback_actuator(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

