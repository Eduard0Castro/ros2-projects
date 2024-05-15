#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotStation: public rclcpp::Node{
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        
        void publisher_news(){
            example_interfaces::msg::String msg;
            msg.data = std::string("Tá saindo da jaula o monstro ") + robot_name;
            publisher_->publish(msg);
        }

        std::string robot_name;

    public:

        RobotStation():Node("robot_station"){
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
            this->declare_parameter("robot_name", "EJSC");
            robot_name = this->get_parameter("robot_name").as_string();
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                             std::bind(&RobotStation::publisher_news, this));
            RCLCPP_INFO(this->get_logger(), "Robot Station News has been started");
        }

};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}