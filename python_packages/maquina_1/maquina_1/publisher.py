import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class MachineNumberOne(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number_pub", 2)
        self.declare_parameter("timer_period_sec", 0.5)

        self.number = self.get_parameter("number_pub").value
        self.timer_to_pub = self.get_parameter("timer_period_sec").value
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(self.timer_to_pub, self.number_publisher)
        self.get_logger().info("Number publisher has been initialized")


    def number_publisher(self):
        msg = Int64()
        msg.data = self.number
        self.publisher_.publish(msg)

    

def main(args = None):
    rclpy.init(args=args)
    node = MachineNumberOne()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
