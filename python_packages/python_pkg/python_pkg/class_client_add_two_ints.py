#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class ClientAddTwoInts(Node):
    def __init__(self):
        super().__init__("client_add_two_ints")

    def call_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        request = AddTwoInts.Request()

        request.a = a
        request.b = b

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for server Add Two Ints")

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call, a=a, b=b))

    def callback_call(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")

        except Exception as e:
            self.get_logger().error("Service call error %r" %(e,))


def main(args = None):
    rclpy.init(args=args)
    node = ClientAddTwoInts()

    i1 = 43
    i2 = 54

    for i in range (0, 10):
        node.call_server(i1 + i, i2 + i)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
