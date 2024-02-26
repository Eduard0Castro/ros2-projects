#include "rclcpp/rclcpp.hpp"
#include "robot_interface/msg/hardware_status.hpp"

class HardwareStatusNode: public rclcpp::Node{

    private:

        rclcpp::Publisher<robot_interface::msg::HardwareStatus>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void hardware_status_publisher(){
            robot_interface::msg::HardwareStatus msg;
            msg.temperature = 42;
            msg.arm_check = true;
            msg.message = "Mamute's sensor is working";
            
            publisher_->publish(msg);
        }

    public:

        HardwareStatusNode():Node("hardware_status"){
            publisher_ = this->create_publisher<robot_interface::msg::HardwareStatus>("ju_mamute_sensor", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                             std::bind(&HardwareStatusNode::hardware_status_publisher, this));
            RCLCPP_INFO(this->get_logger(), "Hardware Status Node has been initialized");
        }


};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}