#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from robot_interface.srv import ResistanceSystem
from functools import partial

class ResistanceClient(Node):
    def __init__(self):
        super().__init__("resistance_client")
        
        
    def call_server(self, tension, current):
        self.client_ = self.create_client(ResistanceSystem, "resistance_measure")
        request = ResistanceSystem.Request()
        request.tension = tension
        request.current = current

        while not self.client_.wait_for_service(1):
            self.get_logger().warn("Waiting for service")



        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_call))



    def callback_call(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Resistance: {response.resistance}")

        except Exception as ex:
            self.get_logger().error("ERROR: Except!!!%r" %(ex,))


    

def main(args = None):
    rclpy.init(args=args)
    node = ResistanceClient()
    node.call_server(3.3, 0.125)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()