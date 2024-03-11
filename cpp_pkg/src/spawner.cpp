#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "project_turtle_interface/srv/dead_turtle.hpp"
#include "project_turtle_interface/msg/turtle_array.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

class Spawner: public rclcpp::Node{

    private:
        rclcpp::Publisher<project_turtle_interface::msg::TurtleArray>::SharedPtr pub_list;
        rclcpp::Service<project_turtle_interface::srv::DeadTurtle>::SharedPtr server;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<project_turtle_interface::msg::Turtle> alive_turtles;
        std::vector<std::shared_ptr<std::thread>> spawn_thread_;
        std::vector<std::shared_ptr<std::thread>> kill_thread_;

        void dead_server_callback(project_turtle_interface::srv::DeadTurtle::Request::SharedPtr request,
                                  project_turtle_interface::srv::DeadTurtle::Response::SharedPtr response){
            
            std::string name = request->name;
            
            try{
                this->call_kill_server(name);
                for (int i = 0; i < alive_turtles.size(); i++){
                    if (alive_turtles[i].name == name){
                        alive_turtles.erase(alive_turtles.begin() + i);
                        break;
                    }
                }

            }

            catch (std::exception &ex){

            }

        }


    public:

        Spawner(): Node ("spawner"){
            timer_ = this->create_wall_timer(std::chrono::seconds(4), 
                                             std::bind(&Spawner::spawn_server_inter, this));
            server = this->create_service<project_turtle_interface::srv::DeadTurtle>("dead_turtle", 
                                          std::bind(&Spawner::dead_server_callback, this,
                                          std::placeholders::_1, std::placeholders::_2));

            pub_list = this->create_publisher<project_turtle_interface::msg::TurtleArray>("alive_turtles", 10);

        }

        void spawn_server_inter(){
            double x = float(rand()%100/10);
            double y = float(rand()%100/10);
            double theta = 0.0;
            spawn_thread_.push_back(std::make_shared<std::thread>(std::bind(&Spawner::call_spawn_server,
                                                                    this, x, y, theta)));
        }

        void call_spawn_server(double x, double y, double theta){
            auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

            while(!client->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_WARN(this->get_logger(), "Waiting for service");

            request->x = x;
            request->y = y;
            request->theta = theta;

            auto future = client->async_send_request(request);

            try{
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "New turtle is in the map");
                project_turtle_interface::msg::Turtle msg;
                msg.name = response->name;
                msg.x = x;
                msg.y = y;
                msg.theta = theta;
                
                alive_turtles.push_back(msg);
                this->publish_list();

            }

            catch (std::exception &ex){
                
                RCLCPP_ERROR(this->get_logger(), "ERROR: %s", ex.what());

            }

        }

        void publish_list(){
            project_turtle_interface::msg::TurtleArray msg;
            msg.turtles = alive_turtles;
            pub_list->publish(msg);

        }



        void call_kill_server(std::string name){
            auto client = this->create_client<turtlesim::srv::Kill>("kill");
            auto request = std::make_shared<turtlesim::srv::Kill::Request>();

            while (!client->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_WARN(this->get_logger(), "Waiting for service");

            request->name = name;
            

            auto future = client->async_send_request(request);
            
        }

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Spawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}