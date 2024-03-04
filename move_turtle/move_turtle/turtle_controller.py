import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from project_turtle_interface.msg import TurtleArray
from project_turtle_interface.srv import DeadTurtle
from functools import partial
from math import pi, sqrt, atan2


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle_to_catch = None
        self.pose = None
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.sub_pose = self.create_subscription(Pose, "turtle1/pose", self.sub_pose_callback, 10)
        self.sub_alive_turtles = self.create_subscription(TurtleArray, "alive_turtles", 
                                                          self.sub_alive_turtles_callback, 10)
        self.timer_ = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Controller has been initialized")

    def sub_pose_callback(self, msg):
        self.pose = msg

    def sub_alive_turtles_callback(self, msg):
        turtles = msg.turtles
        turtle_closest = None
        self.distance_closest = None

        if len(turtles) > 0:

            for turtle in turtles:
                dist_x = turtle.x - self.pose.x
                dist_y = turtle.y - self.pose.y
                dist = sqrt(dist_x**2 + dist_y**2)
                if turtle_closest == None or dist < self.distance_closest:
                    turtle_closest = turtle
                    self.distance_closest = dist 

            self.turtle_to_catch = turtle_closest

    def control_loop(self):
        if self.pose == None or self.turtle_to_catch == None:
            return
        
        dist_x = self.turtle_to_catch.x - self.pose.x
        dist_y = self.turtle_to_catch.y - self.pose.y
        distance = sqrt(dist_x**2 + dist_y**2)
        theta_goal = atan2(dist_y, dist_x)
        msg = Twist()
        self.theta = theta_goal - self.pose.theta

        if self.theta > pi:
            self.theta -= 2*pi
        elif self.theta < -pi:
            self.theta += 2*pi

        msg.linear.x = 2*distance
        msg.angular.z = 6*self.theta

        if distance < 0.4:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_kill_server()
            self.turtle_to_catch = None

        self.publisher_.publish(msg)
        

    def call_kill_server(self):
        client = self.create_client(DeadTurtle, "dead_turtle")
        request = DeadTurtle.Request()
        request.name = self.turtle_to_catch.name

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for service")

        future = client.call_async(request)
        future.add_done_callback(partial(self.kill_callback))

    def kill_callback(self, future):
        try:
            response = future.result()
            success = response.success
            if success == True:
                self.get_logger().info(f"{self.turtle_to_catch.name} has been killed")
        
        except Exception as ex:
            self.get_logger().error("Service call failed: %r" %(ex,))




def main(args = None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
