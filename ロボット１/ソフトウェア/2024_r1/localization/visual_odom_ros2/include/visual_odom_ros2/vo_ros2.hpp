#ifndef VO_ROS2_HPP_
#define VO_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>

#include "vo.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace visual_odom_ros2
{
    class VisualOdomROS2 : public rclcpp::Node
    {
        public:
        explicit VisualOdomROS2(const rclcpp::NodeOptions option=rclcpp::NodeOptions());

        void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rpy_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
        void timer_callback();
        
        private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rpy_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<VisualOdometer> vo;

        geometry_msgs::msg::Vector3 rpy;

        std::string frame_id_param;
        bool rgb_flag, depth_flag;
        
    };
}

#endif