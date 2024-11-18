#ifndef WAYPOINT_PLANNER_HPP_
#define WAYPOINT_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace waypoint_planner
{
    class WayPointPlanner : public rclcpp::Node
    {
        public:
        explicit WayPointPlanner(const rclcpp::NodeOptions option=rclcpp::NodeOptions());

        void topic_callback(const std_msgs::msg::Float32::SharedPtr msg);

        private:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr controller_subscriber_;

        geometry_msgs::msg::PoseStamped home;
        geometry_msgs::msg::PoseStamped first_pose;
        geometry_msgs::msg::PoseStamped second_pose;
        geometry_msgs::msg::PoseStamped third_pose;
    };
}

#endif