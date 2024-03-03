import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from project_turtle_interface.msg import TurtleArray
from math import pi, sqrt, atan2



class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.sub_pose = self.create_subscription(Pose, "turtle1/pose", self.sub_pose, 10)
        self.sub_alive_turtles = self.create_subscription(TurtleArray, "alive_turtles", 
                                                          self.sub_alive_turtles_callback, 10)
        self.turtle_to_catch = None

    def sub_pose_callback(self, msg):
        self.pose = msg

    def sub_alive_turtles_callback(self, msg):
        turtles = msg.turtles
        turtle_closest = None
        distance_closest = None

        if len(turtles) > 0:

            for turtle in turtles:
                dist_x = turtle.x - self.pose.x
                dist_y = turtle.y - self.pose.y
                dist = sqrt(dist_x**2 + dist_y**2)
                if turtle_closest == None or dist < distance_closest:
                    turtle_closest = turtle
                    distance_closest = dist

            self.turtle_to_catch = turtle_closest

        

    






def main(args = None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
