import rclpy
from rclpy.node import Node
from led_interface.srv import LedState
from functools import partial

class BatteryState(Node):
    def __init__(self):
        super().__init__("battery_state")
        self.battery_state = "full"
        self.last_time_state_changed = self.get_current_time()
        self.timer_ = self.create_timer(0.1, self.check_battery_state)
        self.get_logger().info("Battery state has been initialized")

    def get_current_time(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()

        return secs + nsecs/1000000000.0
    
    def check_battery_state(self):
        time_now = self.get_current_time()
        if self.battery_state == "full":
            if time_now - self.last_time_state_changed > 4:
                self.battery_state = "empty"
                self.call_server(1, self.battery_state)
                self.last_time_state_changed = time_now
                self.get_logger().info("Battery is empty: charging...")

        else:
            if time_now - self.last_time_state_changed > 6:
                self.battery_state = "full"
                self.call_server(1, self.battery_state)
                self.last_time_state_changed = time_now
                self.get_logger().info("Battery is now full again")
             

    def call_server(self, led_number: int, state: str):
       
        """
        led_number: led from 1 to 3
        state: string that can be "full" or "empty"
        """

        self.client_ = self.create_client(LedState, "set_led")
        request = LedState.Request()
        request.led_number = led_number
        request.state = state

        while not self.client_.wait_for_service(1):
            self.get_logger().warn("Waiting for service")

        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_call))

    def callback_call(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Success: {response.success}")

        except Exception as ex:
            self.get_logger().error("Something wrong is not correct %r" %(ex,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryState()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()