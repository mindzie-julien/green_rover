#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import action_class as actions

def main(args=None):
    rclpy.init(args=args)

    motion_action_server = actions.MotionActionServer()
    motion_action_server.get_logger().info('motion_server online')
    rclpy.spin(motion_action_server)


if __name__ == '__main__':
    main()
