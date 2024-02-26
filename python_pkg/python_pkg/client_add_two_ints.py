#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args = None):
    rclpy.init(args=args)
    node = Node("client_add_two_ints")
    client = node.create_client(AddTwoInts, "add_two_ints")

    while not client.wait_for_service(1):
        node.get_logger().warn("Waiting for Server add_two_ints")

    request = AddTwoInts.Request()
    request.a = 9
    request.b = 13
    for i in range(0, 500, 10):
        request.a += i
        request.b += i

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future=future)

        try:
            response = future.result()
            node.get_logger().info(f"{request.a} + {request.b} = {response.sum}")

        except Exception as e:
            node.get_logger().error("Service call error %r" %(e,))


    rclpy.shutdown()


if __name__ == "__main__":
    main()
