#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interface.srv import ResistanceSystem

class ResistanceServer(Node):
    def __init__(self):
        super().__init__("resistance_server")
        self.server_ = self.create_service(ResistanceSystem, "resistance_measure", 
                                           self.resistance_callback)
        self.get_logger().info("Resistance server has been initialized")
    

    def resistance_callback(self, request, response):
        response.resistance = request.tension/request.current

        self.get_logger().info(f"Resistance: {response.resistance}")

        return response


def main(args = None):
    rclpy.init(args=args)
    node = ResistanceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

