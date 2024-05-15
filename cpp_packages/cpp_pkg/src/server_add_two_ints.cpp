#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"


class AddTwoInts: public rclcpp::Node{

    private:
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server;

    public:

        AddTwoInts(): Node("server_add_two_ints"){

            server = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", 
                                            std::bind(&AddTwoInts::add_two_ints_callback, this, std::placeholders::_1,
                                            std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Server Add Two Ints has been initialized");

        }

        void add_two_ints_callback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request, 
                                    const example_interfaces::srv::AddTwoInts::Response::SharedPtr response){

            response->sum = request->a + request->b;
            int resp = response->sum;
            int a = request->a;
            int b = request->b;
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, resp);
 

        }


};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoInts>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;


}