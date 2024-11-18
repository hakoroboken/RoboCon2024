#ifndef FIELD_SIM_HPP_
#define FIELD_SIM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <chrono>

using namespace std::chrono_literals;

namespace field_sim
{
    class FieldSim : public rclcpp::Node
    {
        public:
        explicit FieldSim(const rclcpp::NodeOptions option = rclcpp::NodeOptions());

        void timer_callback();

        private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::Pose robot_pose;
        visualization_msgs::msg::MarkerArray ma;
    };
}

#endif