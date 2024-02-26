#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"



int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("client_add_two_ints");
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();


    client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    while(!client->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(node->get_logger(), "Waiting for service");
    }

    request->a = 9;
    request->b = 80;

    for (int i = 0, j = 10; i <= 50; i += 1){
        request->a += j;
        request->b += j;

        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
            RCLCPP_INFO(node->get_logger(), "%d + %d = %d", 
                                            (int)request->a, (int)request->b, 
                                            (int)future.get()->sum);

        else
            RCLCPP_ERROR(node->get_logger(), "Something is not correct with the comunication");
        
    }


    rclcpp::shutdown();
    return 0;

}