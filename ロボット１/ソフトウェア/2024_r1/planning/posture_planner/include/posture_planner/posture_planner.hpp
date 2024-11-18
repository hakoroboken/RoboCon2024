#ifndef POSTURE_PLANNER
#define POSTURE_PLANNER

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>

#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace posture_planner
{
    class PosturePlanner : public rclcpp::Node
    {
        public:
        explicit PosturePlanner(const rclcpp::NodeOptions option = rclcpp::NodeOptions());

        void current_sub_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
        void target_sub_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr current_sub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_vel_publisher;

        rclcpp::TimerBase::SharedPtr timer_;

        float current_yaw, target_yaw;
        bool current_flag, target_flag;

        // Parameters
        float threshold;
        bool enable_reverse;
    };
}

#endif