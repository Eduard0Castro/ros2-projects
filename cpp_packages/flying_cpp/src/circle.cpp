#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"


class Circle: public rclcpp::Node{

    private:

        rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr vel_pub;
        std::vector<std::shared_ptr<std::thread>> thread;

    public:

        Circle(): Node("circle"){

            vel_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
            thread.push_back(std::make_shared<std::thread>(std::bind(&Circle::set_mode_call_server, this, "GUIDED")));
            RCLCPP_INFO(this->get_logger(), "Circle node has been initialized");
        }

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

        void set_mode_call_server(std::string mode){
            auto client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();

            while (!client->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_INFO(this->get_logger(), "Waiting for service");

            request->base_mode = 0;
            request->custom_mode = mode;

            auto future = client->async_send_request(request);

            try{
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Mode set success: %d", response->mode_sent);
                thread.push_back(std::make_shared<std::thread>(std::bind(&Circle::arm_call_server, this)));


            }

            catch(std::exception &ex){
                RCLCPP_ERROR(this->get_logger(), "ERROR: %s", ex.what());
            }

        }

        void arm_call_server(){
            auto client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();

            while (!client->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_INFO(this->get_logger(), "Waiting for service");

            request->value = true;

            auto future = client->async_send_request(request);

            try{
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Arming: %d", response->success);
                rclcpp::sleep_for(std::chrono::seconds(2));
                thread.push_back(std::make_shared<std::thread>(std::bind(&Circle::takeoff_call_server, this)));

            }

            catch(std::exception &ex){
                RCLCPP_ERROR(this->get_logger(), "ERROR: %s", ex.what());
            }

        }

        void takeoff_call_server(){
            auto client = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
            auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

            while (!client->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_INFO(this->get_logger(), "Waiting for service");

            request->min_pitch = 0;
            request->latitude = 0;
            request->longitude = 0;
            request->altitude = 7;

            auto future = client->async_send_request(request);

            try{
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Takeoff %d meters: %d", (int)request->altitude, response->success);
                rclcpp::sleep_for(std::chrono::seconds(10));
                this->publish_vel_msg();

            }

            catch(std::exception &ex){
                RCLCPP_ERROR(this->get_logger(), "ERROR: %s", ex.what());
            }

        }

        void publish_vel_msg(){
            mavros_msgs::msg::PositionTarget msg;
            double t_now = this->get_current_time();
            double t_start = this->get_current_time();
            double duration = 15.0;
            
            msg.coordinate_frame = 8;
            msg.type_mask = 1479;
            msg.velocity.x = 2;
            msg.velocity.y = 0;
            msg.velocity.z = 0;
            msg.yaw_rate = 1;
            rclcpp::Rate rate(30);

            while(t_now <= t_start + duration){
                vel_pub->publish(msg);
                rate.sleep();
                t_now = this->get_current_time();

            }

            msg.velocity.x = 0;
            msg.yaw_rate = 0;
            vel_pub->publish(msg);
        }

};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Circle>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}