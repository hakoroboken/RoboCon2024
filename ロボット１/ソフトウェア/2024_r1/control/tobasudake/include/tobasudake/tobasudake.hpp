#ifndef TOBASUDAKE_HPP_
#define TOBASUDAKE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace tobasudake
{
    class TobasuDake : public rclcpp::Node
    {
        public:
        explicit TobasuDake(const rclcpp::NodeOptions& option=rclcpp::NodeOptions());

        void x_topic_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void y_topic_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void rotation_topic_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void machine_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr x_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rotation_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr machine_sub;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::Twist prev_vector;
        geometry_msgs::msg::Twist target_vector;
        float m;
        double smooth_gain;
        bool x_flag, y_flag, rotation_flag;
    };
}

#endif