#ifndef DUALSENES_ROS2_NODE_HPP_
#define DUALSENES_ROS2_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace dualsense_ros2
{
    class DualSenseROS2 : public rclcpp::Node
    {
        public:
        DualSenseROS2(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());
        void timer_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        
        private:
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr linear_x_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr linear_y_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_z_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_1;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_2;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_3;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_4;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_5;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_6;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_7;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_8;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr f_publisher_9;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    };
}

#endif