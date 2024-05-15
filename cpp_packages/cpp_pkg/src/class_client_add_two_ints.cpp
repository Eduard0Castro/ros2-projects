#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"


class AddTwoIntsClient: public rclcpp::Node{

    private:

        std::thread thread_;

    public:

        AddTwoIntsClient():Node("client_add_two_ints"){
            thread_ = std::thread(std::bind(&AddTwoIntsClient::call_server, this, 13, 22));
        }

        void call_server(int a, int b){
            auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            while (!client->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_WARN(this->get_logger(), "Waiting for service");

            request->a = a;
            request->b = b;
            
            auto future = client->async_send_request(request);

            try 
            {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, (int)response->sum);
            }
            
            catch(std::exception &ex){
                RCLCPP_ERROR(this->get_logger(), "ERROR: Service call failed");
            }
        }

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;


}