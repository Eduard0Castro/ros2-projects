#include "rclcpp/rclcpp.hpp"
#ifndef MYNODE_HPP
#define MYNODE_HPP

class MyNode: public rclcpp::Node{

    private:
        int cont = 0;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        MyNode(): Node("cpp_test"){
            RCLCPP_INFO(this->get_logger(), "Hello cpp_node from class");
            timer_= this->create_wall_timer(std::chrono::seconds(1), 
                                            std::bind(&MyNode::timer_callback, this));
        }

        void timer_callback(){
            cont ++;
            RCLCPP_INFO(this->get_logger(), "Hello again %d", cont);
        }
};

#endif