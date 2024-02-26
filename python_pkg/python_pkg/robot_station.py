#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotStation(Node):
    def __init__(self):
        super().__init__("robot_station")
        self.cont = 0
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publisher_news)
        self.get_logger().info("\033[32mRobot Station News has been started\033[m]")


    def publisher_news(self):
        self.cont += 10
        msg = String()
        msg.data = f"\033[33mTá saindo da jaula o monstro {self.cont}\033[m"
        self.publisher_.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    node = RobotStation()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()