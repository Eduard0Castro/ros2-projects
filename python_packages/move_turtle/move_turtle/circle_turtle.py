import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class MoveTurtle(Node):
    def __init__(self):
        super().__init__("circle_turtle")
        self.pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.25, self.move_pub)


    def move_pub(self):
        msg = Twist()
        msg.linear.x = 10.0
        msg.angular.z = 11.0
        self.pub.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

