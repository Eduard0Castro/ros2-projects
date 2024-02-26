#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"


class MachineNumberTwo: public rclcpp::Node{
    private:
        int cont = 0;
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
        rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;

    public:

        MachineNumberTwo():Node("number_counter"){
            
            subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, 
                                                    std::bind(&MachineNumberTwo::number_counter_callback, 
                                                    this, std::placeholders::_1));
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("counter", 10);
            server_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
                                            std::bind(&MachineNumberTwo::reset_counter_callback, this, 
                                            std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Number Counter has been initialized");

        }

        void number_counter_callback(example_interfaces::msg::Int64::SharedPtr msg){
                
                example_interfaces::msg::Int64 new_msg;
                int received = msg->data;

                new_msg.data = cont;
                publisher_->publish(new_msg);
                cont += msg->data;

                RCLCPP_INFO(this->get_logger(), "%d", received);
        }

        void reset_counter_callback(example_interfaces::srv::SetBool::Request::SharedPtr request,
                                    example_interfaces::srv::SetBool::Response::SharedPtr response)
        {
            bool reset = request->data;

            if (reset){
                cont = 0;
                response->success = true;
                response->message = "Counter has been reseted";
                RCLCPP_INFO(this->get_logger(), "Counter has been reseted");

            }

            else {
                RCLCPP_INFO(this->get_logger(), "Nothing to do");
                response->success = false;
                response->message = "Operation result: false";
            }
                bool sucess = response->success;
                RCLCPP_INFO(this->get_logger(), "Operation: %s", sucess ? "true":"false");

        }

};


int main(int argc, char **argv){
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MachineNumberTwo>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}