import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from random import uniform
from math import pi
from functools import partial
from project_turtle_interface.srv import DeadTurtle
from project_turtle_interface.msg import Turtle, TurtleArray


class TurtleSpwaner (Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.timer = self.create_timer(4, self.call_spawn_server)
        self.server = self.create_service(DeadTurtle, "dead_turtle", self.server_turtle_kill)
        self.alive_turtles__ = list()
        self.pub_alive_turtles = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.get_logger().info("Turtle Spawner has been initialized teste")


    def call_spawn_server(self):
        client = self.create_client(Spawn, "spawn")

        self.spawn_x = uniform(1, 10)
        self.spawn_y = uniform(1, 10)
        self.spawn_theta = uniform(-pi, pi)

        request = Spawn.Request()
        request.x = self.spawn_x
        request.y = self.spawn_y
        request.theta = self.spawn_theta

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for service")

        future = client.call_async(request)
        future.add_done_callback(partial(self.spawn_client_callback))
        

    def spawn_client_callback(self, future):
        try:
            self.response = future.result()
            self.get_logger().info(f"Name: {self.response.name}")
            msg = Turtle()
            msg.name = self.response.name
            msg.x = self.spawn_x
            msg.y = self.spawn_y
            msg.theta = self.spawn_theta
            self.alive_turtles__.append(msg)
            self.pub_alive_turtles_func()

        except Exception as ex:
            self.get_logger().error("Something wrong is not right: %r" %(ex,))


    def pub_alive_turtles_func(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles__
        self.pub_alive_turtles.publish(msg)
    

    def call_kill_server(self, turtle_name):
        client = self.create_client(Kill, "kill")
        request = Kill.Request()
        self.name = turtle_name
        request.name = self.name

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for service")

        client.call_async(request)

        for i, turtle in enumerate(self.alive_turtles__):
            if turtle.name == turtle_name:
                del self.alive_turtles__[i]
                break



    def server_turtle_kill(self, request, response):
        name = request.name
        
        try: 
            self.call_kill_server(name)
            response.success = True

        except Exception as ex:
            self.get_logger().error("ERROR: %r" %(ex,))
            response.success = False


        return response

    


def main(args = None):
    rclpy.init(args=args)
    node = TurtleSpwaner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()