#include "rclcpp/rclcpp.hpp"
#include "led_interface/msg/led_trigger.hpp"
#include "led_interface/srv/led_state.hpp"

class SetLedServer: public rclcpp::Node{

    private:

        rclcpp::Publisher<led_interface::msg::LedTrigger>::SharedPtr publisher_;
        rclcpp::Service<led_interface::srv::LedState>::SharedPtr server_;
        rclcpp::TimerBase::SharedPtr timer_;
        led_interface::msg::LedTrigger msg;

        void call_server(led_interface::srv::LedState::Request::SharedPtr request, 
                    led_interface::srv::LedState::Response::SharedPtr response){

            int led_number = request->led_number;
            std::string state = request->state;
            int size = msg.leds.size();

            try{
                if (led_number <= size && led_number > 0){
                    if (state == "full"||state == "empty"){
                        if (state == "empty"){
                            msg.leds[led_number-1] = true;
                        }
                        else if (state == "full"){
                            msg.leds[led_number-1] = false;
                        }

                        RCLCPP_INFO(this->get_logger(), "Led %d modificated", led_number);
                        response->success = true;
                        led_panel_publish();
                    }

                    else{
                        response->success = false;
                        RCLCPP_ERROR(this->get_logger(), "State argument invalid value. It must be empty or full");
                    }
                }
                else {   

                    RCLCPP_ERROR(this->get_logger(), "ERROR: led_number out of the range");
                    response->success = false;
                }
            }

            catch(std::exception &ex){
                RCLCPP_ERROR(this->get_logger(), "Something wrong is not correct");
                response->success = false;
            }
        }

        void led_panel_publish(){
            publisher_->publish(msg);
        }

    public:

        SetLedServer(): Node("set_led_server"){

            server_ = this->create_service<led_interface::srv::LedState>("set_led", 
                                            std::bind(&SetLedServer::call_server, this,
                                            std::placeholders::_1, std::placeholders::_2));

            publisher_ = this->create_publisher<led_interface::msg::LedTrigger>("led_panel_trigger", 10);
            msg.leds = {false, false, false};
            timer_ = this->create_wall_timer(std::chrono::seconds(4), std::bind(&SetLedServer::led_panel_publish, this));
            RCLCPP_INFO(this->get_logger(), "Led Panel Server has been initialized");
        }
};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetLedServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}
