#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class MachineNumberTwo(Node):

    def __init__(self):
        self.cont = 0
        super().__init__("number_counter")
        self.subscriber_ = self.create_subscription(Int64, "number",
                                                     self.number_counter_callback, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.server_ = self.create_service(SetBool, "reset_counter", self.reset_counter_callback)
        self.get_logger().info("Number counter has been initialized")

    def number_counter_callback(self, msg=None):

        msg2 = Int64()
        self.get_logger().info(str(msg.data))
        msg2.data = self.cont
        self.publisher_.publish(msg2)
        self.cont += 1

    def reset_counter_callback(self, request, response):
        
        reset = request.data

        if reset:
            self.cont = 0
            response.success = True
            response.message = "Counter has been reseted"
            self.get_logger().info(str(response.message))

        else:
            response.success = False
            response.message = "Nothing to do"
            self.get_logger().info(str(response.message))

        self.get_logger().info("Operation complete: result =" + str(response.success))

        return response
            
        

def main(args = None):
    
    rclpy.init(args=args)
    node = MachineNumberTwo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()