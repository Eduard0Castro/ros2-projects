#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"


class Smartphone: public rclcpp::Node{

    private:
        
        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;

    public:

        Smartphone(): Node("smartphone"){
            subscriber_ = this->create_subscription<example_interfaces::msg::String>("robot_news", 
                                                    10, std::bind(&Smartphone::callback_robot_news,
                                                    this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Smartphone node has been started");
        }

        void callback_robot_news(example_interfaces::msg::String::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        }

        

};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Smartphone>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}