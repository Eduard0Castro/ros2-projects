#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interface.msg import HardwareStatus


class HardwareStatusPublisher(Node):

    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.publisher_ = self.create_publisher(HardwareStatus, "carianis_sensor", 10)
        self.timer_ = self.create_timer(0.5, self.carianis_sensor)
        self.get_logger().info("Robot Hardware Publisher has been started")
 
    def carianis_sensor(self):
        msg = HardwareStatus()
        msg.arm_check = True
        msg.temperature = 42
        msg.message = "Sensor is working"
        self.publisher_.publish(msg)
        

def main(args = None):
    rclpy.init(args=args)
    node = HardwareStatusPublisher()    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()