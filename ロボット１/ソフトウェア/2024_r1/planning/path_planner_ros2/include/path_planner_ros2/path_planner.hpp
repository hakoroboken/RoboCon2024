#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <chrono>
#include <vector>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace path_planner
{
    class PathPlanner : public rclcpp::Node
    {
        public:
        explicit PathPlanner(const rclcpp::NodeOptions option = rclcpp::NodeOptions());
        void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void timer_callback();

        private:
        tf2::Vector3 getEuler(const geometry_msgs::msg::Quaternion q)
        {
            tf2::Vector3 v;
            v.setW(q.w);
            v.setX(q.x);
            v.setY(q.y);
            v.setZ(q.z);

            return v;
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscriber;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::PoseStamped::SharedPtr current_;
        geometry_msgs::msg::PoseStamped::SharedPtr target_;

        std::string frame_id_param;
        double step_num_param;
    };
}

#endif