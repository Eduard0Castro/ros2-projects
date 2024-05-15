#include "rclcpp/rclcpp.hpp"
#include "robot_interface/srv/resistance_system.hpp"

class ResistanceClient: public rclcpp::Node{

    private:

        std::thread thread_;
        
    public:

        ResistanceClient(float tension, float current): Node("resistance_client"){
            thread_ = std::thread(std::bind(&ResistanceClient::call_server, this, tension, current));
        }

        void call_server(float tension, float current){

            auto client = this->create_client<robot_interface::srv::ResistanceSystem>("resistance_measure");
            auto request = std::make_shared<robot_interface::srv::ResistanceSystem::Request>();

            while (!client->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_WARN(this->get_logger(), "Waiting for service");

            request->tension = tension;
            request->current = current;

            auto future = client->async_send_request(request);

            try{

                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Resistance: %.2f ohms", (float)response->resistance);

            }

            catch(std::exception &ex){

                RCLCPP_ERROR(this->get_logger(), "Exception occurred");

            }
        }

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResistanceClient>(3.3, 0.125);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}