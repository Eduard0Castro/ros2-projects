#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class MachineOne: public rclcpp::Node{


    private:

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;

    public:

        MachineOne():Node("number_publisher"){
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                             std::bind(&MachineOne::number_publisher, this));
            RCLCPP_INFO(this->get_logger(), "Number Publisher has been initialized");
        }

        void number_publisher(){
            example_interfaces::msg::Int64 msg;
            msg.data = 43;
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
