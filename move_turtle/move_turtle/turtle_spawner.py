import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from random import randint
from functools import partial
from project_turtle_interface.srv import DeadTurtle


class TurtleSpwaner (Node):
    def __init__(self):
        super().__init__("turtle_spwaner")
        self.timer = self.create_timer(4, self.call_spawn_server)
        self.get_logger().info("Turtle Spawner has been initialized")
        self.server = self.create_service(DeadTurtle, "dead_turtle", self.server_turtle_kill)
        

    def call_spawn_server(self):
        client = self.create_client(Spawn, "spawn")
        request = Spawn.Request()
        request.x = float(randint(1, 10))
        request.y = float(randint(1, 10))
        request.theta = float(randint(-3, 3))

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for service")


        future = client.call_async(request)
        future.add_done_callback(partial(self.spawn_client_callback))
        

    def spawn_client_callback(self, future):
        try:
            self.response = future.result()
            self.get_logger().info(f"Name: {self.response.name}")


        except Exception as ex:
            self.get_logger().error("Something wrong is not right: %r" %(ex,))

    def call_kill_server(self):
        client = self.create_client(Kill, "kill")
        request = Kill.Request()
        self.name = "turtle2"
        request.name = self.name

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for service")


        client.call_async(request)

    def server_turtle_kill(self, request, response):
        _=request
        self.call_kill_server()
        response.name = self.name
        return response

    
def main(args = None):
    rclpy.init(args=args)
    node = TurtleSpwaner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()