#include "rclcpp/rclcpp.hpp"
#include "robot_interface/srv/resistance_system.hpp"

class ResistanceServer: public rclcpp::Node{

    private:

        rclcpp::Service<robot_interface::srv::ResistanceSystem>::SharedPtr server_;

        void resistance_callback(robot_interface::srv::ResistanceSystem::Request::SharedPtr request, 
                                 robot_interface::srv::ResistanceSystem::Response::SharedPtr response)
        {
            response->resistance = request->tension/request->current;
            float resistance = response->resistance;

            RCLCPP_INFO(this->get_logger(), "Resistance: %.2f", resistance);
        }

    public:

        ResistanceServer():Node("resistance_server"){

            server_ = this->create_service<robot_interface::srv::ResistanceSystem>("resistance_measure",
                                            std::bind(&ResistanceServer::resistance_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Resistance Server has been initialized");
        }

};


int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResistanceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

}