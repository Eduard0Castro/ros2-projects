#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interface.msg import HardwareStatus

class SubsHardwareStatus(Node):
    def __init__(self):
        super().__init__("subs_hardware_status")
        self.subscriber_ = self.create_subscription(HardwareStatus, "carianis_sensor", 
                                                    self.cariani_callback, 10)
        self.get_logger().info("Subscriber Hardware Status has been initialized")


    def cariani_callback(self, msg):
        temperature = msg.temperature
        arm = msg.arm_check

        self.get_logger().info(f"Datas received from sensor: \n{temperature}°\nArm: {arm}")


def main(args = None):
    rclpy.init(args=args)
    node = SubsHardwareStatus()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":

    main()
