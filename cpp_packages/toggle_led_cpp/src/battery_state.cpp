#include "rclcpp/rclcpp.hpp"
#include "led_interface/srv/led_state.hpp"

class BatteryState: public rclcpp::Node{

    private:

        std::thread thread_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string battery_state;
        float last_time_state_changed;

        double get_current_time(){

            auto currentTimePoint = std::chrono::system_clock::now();

            // Convertendo o tempo para um formato de tempo desde o epoch
            auto duration = currentTimePoint.time_since_epoch();

            // Obtendo o tempo em segundos e nanossegundos
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(duration);
            auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - secs);

            std::cout << secs.count()<< std::endl;
            return secs.count();

        }

        void call_server(int led_number, std::string state){
            
            auto client = this->create_client<led_interface::srv::LedState>("set_led");
            auto request = std::make_shared<led_interface::srv::LedState::Request>();

            while (!client->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_WARN(this->get_logger(), "Waiting for service");

            request->led_number = led_number;
            request->state = state;

            auto future = client->async_send_request(request);

            try{
                
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Success: %d", response->success);

            }

            catch (std::exception &ex){
                RCLCPP_ERROR(this->get_logger(), "Exception occurred");
            }
        }


        void check_state(){
            float time_now = get_current_time();
            int led_number = 3;
            if(battery_state == "full"){
                if(time_now - last_time_state_changed > 4){
                    battery_state = "empty";
                    this->call_server(led_number, battery_state);
                    RCLCPP_WARN(this->get_logger(), "Battery is empty... Charging");
                    last_time_state_changed = time_now;
                }
            }

            else{
                    battery_state = "full";
                    this->call_server(led_number, battery_state);
                    RCLCPP_INFO(this->get_logger(), "Battery is full again!");
                    last_time_state_changed = time_now;
            }
        }

    public:

        BatteryState(): Node("battery_node"){

            thread_ = std::thread(std::bind(&BatteryState::call_server, this, 3, "empty"));
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryState::check_state, this));
            battery_state = "full";
            last_time_state_changed = this->get_current_time();
        }
};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryState>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}