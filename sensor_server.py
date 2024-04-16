#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import action_class as actions

def main(args=None):
    rclpy.init(args=args)

    sensor_action_server = actions.SensorActionServer()
    sensor_action_server.get_logger().info('sensor_server online')
    rclpy.spin(sensor_action_server) 

if __name__ == '__main__':
    main()
