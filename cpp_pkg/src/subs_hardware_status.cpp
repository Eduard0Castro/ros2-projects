#include "rclcpp/rclcpp.hpp"
#include "robot_interface/msg/hardware_status.hpp"

class SubsHardwareStatus: public rclcpp::Node{

    private:
        rclcpp::Subscription<robot_interface::msg::HardwareStatus>::SharedPtr subscriber_;


    void mamute_callback(robot_interface::msg::HardwareStatus msg){
        int graus = msg.temperature;
        bool arm = msg.arm_check;

        RCLCPP_INFO(this->get_logger(), "Datas: \nTemperature: %d \nArm: %d", graus, arm);
    }
    
    public:

        SubsHardwareStatus(): Node("subs_hardware_status"){
            subscriber_ = this->create_subscription<robot_interface::msg::HardwareStatus>("ju_mamute_sensor",10, 
                                                        std::bind(&SubsHardwareStatus::mamute_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Subs Hardware Status has been initialized");

        }

};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubsHardwareStatus>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;


}