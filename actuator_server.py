#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import action_class as actions

def main(args=None):
    rclpy.init(args=args)
    
    actuator_action_server = actions.ActuatorActionServer()
    actuator_action_server.get_logger().info('actuator_server online')
    rclpy.spin(actuator_action_server)
    
if __name__ == '__main__':
    main()
