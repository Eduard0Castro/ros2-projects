#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class MachineOne: public rclcpp::Node{


    private:

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
        int number;
        float frequency;

    public:

        MachineOne():Node("number_publisher"){
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
            this->declare_parameter("number_pub", 26);
            this->declare_parameter("frequency_pub", 2);

            number = this->get_parameter("number_pub").as_int();
            frequency = this->get_parameter("frequency_pub").as_int();
            int time = (1/frequency)*1000;
            timer_ = this->create_wall_timer(std::chrono::milliseconds(time), 
                                             std::bind(&MachineOne::number_publisher, this));
            RCLCPP_INFO(this->get_logger(), "Number Publisher has been initialized");
        }

        void number_publisher(){
            example_interfaces::msg::Int64 msg;
            msg.data = number;
            publisher_->publish(msg);
        }
};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MachineOne>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
