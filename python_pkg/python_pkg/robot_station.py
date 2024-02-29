#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotStation(Node):
    def __init__(self):
        super().__init__("robot_station")

        self.declare_parameter("robot_name", "EJSC")
        self.robot_name = self.get_parameter("robot_name").value
        
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publisher_news)
        self.get_logger().info("\033[32mRobot Station News has been started\033[m")


    def publisher_news(self):

        msg = String()
        msg.data = f"\033[33mTá saindo da jaula o monstro {self.robot_name}\033[m"
        self.publisher_.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    node = RobotStation()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()