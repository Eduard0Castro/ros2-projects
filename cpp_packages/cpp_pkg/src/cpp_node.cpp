#include "MyNode.hpp"


int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    // auto node = std::make_shared<rclcpp::Node>("node_from_main");
    // RCLCPP_INFO(node->get_logger(), "Hello from main");
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}