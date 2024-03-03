import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from functools import partial
from random import randint
from math import sqrt, pow, atan2, pi

class EatTurtle(Node):
    def __init__(self):
        super().__init__("eat_turtle")
        self.pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.sub_one = self.create_subscription(Pose, "turtle1/pose", 
                                                self.pose_callback_one,10)
        self.call_spawn_server()

    def call_spawn_server(self):
        request_x = float(randint(1,10))
        request_y = float(randint(1,10))
        request_theta = float(randint(-3, -3))
        self.client = self.create_client(Spawn, "spawn")
        self.request = Spawn.Request()
        self.request.x = request_x
        self.request.y = request_y
        self.request.theta = request_theta

        while not self.client.wait_for_service(1):
            self.get_logger().warn("Waiting for service")

        future = self.client.call_async(self.request)
        future.add_done_callback(partial(self.callback))

    def callback(self, future):
        try:
            self.response = future.result()
            self.get_logger().info("Name: " + self.response.name)
            self.sub = self.create_subscription(Pose, f"{self.response.name}/pose", 
                                    self.pose_callback_two, 10)
        except Exception as ex:
            self.get_logger().error("ERROR: %r" %(ex,))

    
    def call_kill_server(self):

        self.client = self.create_client(Kill, "kill")
        self.request = Kill.Request()
        self.request.name = self.response.name


        while not self.client.wait_for_service(1):
            self.get_logger().warn("Waiting for service")

        self.client.call_async(self.request)
        self.call_spawn_server()

    def pose_callback_one(self, msg):
        self.pose_x_one = msg.x
        self.pose_y_one = msg.y
        self.theta_one = msg.theta

    def pose_callback_two(self, msg):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.eat_turtle_pub()
        
    def eat_turtle_pub(self):
        msg = Twist()
        dist_x = self.pose_x - self.pose_x_one 
        dist_y = self.pose_y - self.pose_y_one
        dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2))
        destiny = atan2(dist_y, dist_x)
        theta = destiny - self.theta_one

        if theta > pi:
            theta -= 2*pi
        elif theta < -pi:
            theta += 2*pi

        vel_theta = 6*theta
        vel_x = dist*2

        msg.linear.x = vel_x
        msg.angular.z = vel_theta

        self.pub.publish(msg)

        if dist < 0.4:

            self.call_kill_server()

        
def main(args = None):
    rclpy.init(args=args)
    node = EatTurtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()