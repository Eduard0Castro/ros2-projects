#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        self.cont = 0
        super().__init__("python_test_fromclass")
        self.get_logger().info("\033[31mHello ROS 2!!!\033[m")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.cont += 1
        self.get_logger().info("Hello Again {}" .format(self.cont))

def main(args=None):
    rclpy.init(args=args)
    
    #node = Node("Primeira_maneira_de_criar_o_no")
    #node = rclpy.create_node("teste_oh_my_god")
    node = MyNode()
    
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()  