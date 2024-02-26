import rclpy
from rclpy.node import Node
from led_interface.msg import LedTrigger
from led_interface.srv import LedState

class LedServer(Node):
    def __init__(self):
        super().__init__("led_server")
        self.server_ = self.create_service(LedState, "set_led", self.led_callback)
        self.publisher_ = self.create_publisher(LedTrigger, "led_panel_state", 10)
        self.get_logger().info("Server Led has been initialized")
        

    def led_callback(self, request, response):
        msg = LedTrigger()
        msg.leds = [False, False, False]
        led_number = request.led_number
        state = request.state

        self.get_logger().info(f"State: {state} Led number tete: {led_number}")
        try:
            if state == "False":
                pass
            elif state == "True":
                msg.leds[led_number-1] = True

            response.success = True

        except Exception as ex:
            self.get_logger().error("Something wrong is not correct %r" %(ex,))
            response.success = False

        self.publisher_.publish(msg)
        return response


def main(args = None):
    rclpy.init(args=args)
    node = LedServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()