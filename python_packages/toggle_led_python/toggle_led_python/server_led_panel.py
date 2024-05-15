import rclpy
from rclpy.node import Node
from led_interface.msg import LedTrigger
from led_interface.srv import LedState

class LedServer(Node):
    def __init__(self):
        super().__init__("led_server")
        self.msg = LedTrigger()
        self.declare_parameter("led_state", [False, False, True])
        self.msg.leds = self.get_parameter("led_state").value
        self.server_ = self.create_service(LedState, "set_led", self.led_callback)
        self.publisher_ = self.create_publisher(LedTrigger, "led_panel_state", 10)
        self.hugo = ["full", "empty"]
        self.timer_ = self.create_timer(4, self.pub_on_led)
        self.get_logger().info("Server Led has been initialized")

    def pub_on_led(self):
        self.publisher_.publish(self.msg)

    def led_callback(self, request, response):

        led_number = request.led_number
        state = request.state

        self.get_logger().info(f"State: {state} Led number tete: {led_number}")
        try:
            if state in self.hugo:
                if state == "full":
                    self.msg.leds[led_number-1] = False
                elif state == "empty":
                    self.msg.leds[led_number-1] = True

                response.success = True
                self.pub_on_led()

            else:
                response.success = False

        except Exception as ex:
            self.get_logger().error("Something wrong is not correct %r" %(ex,))
            response.success = False

        return response


def main(args = None):
    rclpy.init(args=args)
    node = LedServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()