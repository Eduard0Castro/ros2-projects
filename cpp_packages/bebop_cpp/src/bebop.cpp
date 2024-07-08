#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class BebopTeste: public rclcpp::Node{

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_bebas;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr take_off_pub, land_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::TimerBase::SharedPtr __timer;


    public:

        BebopTeste(): Node("bebop_teste"){

            sub_bebas = this->create_subscription<sensor_msgs::msg::Image>("/bebop/camera/image_raw", 10, 
                                                    std::bind(&BebopTeste::bebas_callback, this, 
                                                    std::placeholders::_1));
            take_off_pub = this->create_publisher<std_msgs::msg::Empty>("/bebop/takeoff", 10);
            land_pub = this->create_publisher<std_msgs::msg::Empty>("/bebop/land", 10);
            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);
            
            take_off_pub->publish(std_msgs::msg::Empty());
            rclcpp::sleep_for(std::chrono::seconds(5));
            land_pub->publish(std_msgs::msg::Empty());
            //__timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BebopTeste::vel_publisher, this));
            RCLCPP_INFO(this->get_logger(), "Bebop cpp teste has been initialized without take_off");


        }

        void bebas_callback(sensor_msgs::msg::Image::SharedPtr image){

            cv_bridge::CvImagePtr cv_image;
            cv::Mat frame;

            try{
                cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
                frame = cv_image->image;
            }

            catch(cv_bridge::Exception& e){
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            cv::imshow("teste", frame);

            if (cv::waitKey(1) == 0x71) {
                sub_bebas.reset();
                cv::destroyAllWindows();               

            }
        }

        void vel_publisher(){

            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.05;
            vel_pub->publish(msg);            
            
        }
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BebopTeste>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}
