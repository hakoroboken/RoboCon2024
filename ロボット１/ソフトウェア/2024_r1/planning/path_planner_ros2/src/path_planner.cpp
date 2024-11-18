#include "path_planner_ros2/path_planner.hpp"

namespace path_planner
{
    PathPlanner::PathPlanner(const rclcpp::NodeOptions option) : Node("PathPlanner", option)
    {
        target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot/pose/target", 
            0, 
            std::bind(&PathPlanner::target_pose_callback, this, _1)
        );

        current_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot/pose/current",
            0,
            std::bind(&PathPlanner::current_pose_callback, this, _1)
        );

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 0);

        timer_ = this->create_wall_timer(1ms, std::bind(&PathPlanner::timer_callback, this));

        target_ = nullptr;
        current_ = nullptr;

        this->declare_parameter("frame_id", "map");
        this->get_parameter("frame_id", frame_id_param);
        this->declare_parameter("step_size", 0.1);
        this->get_parameter("step_size", step_num_param);
    }

    void PathPlanner::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_ = msg;
    }

    void PathPlanner::current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_ = msg;
    }

    void PathPlanner::timer_callback()
    {
        if(target_ != nullptr && current_ != nullptr)
        {
            auto new_path = nav_msgs::msg::Path();
            new_path.header.frame_id = "map";

            auto target_posture = getEuler(target_->pose.orientation);
            auto current_posture = getEuler(current_->pose.orientation);

            auto dx = target_->pose.position.x - current_->pose.position.x;
            auto dy = target_->pose.position.y - current_->pose.position.y;
            auto drotation = target_posture.getZ() - current_posture.getZ();

            auto p2p = std::sqrt(dx*dx + dy*dy);
            int step_num = p2p / step_num_param;

            for(int i = 0; i < step_num; i++)
            {
                auto t = static_cast<double>(i) / step_num;
                auto p = geometry_msgs::msg::PoseStamped();
                p.pose.position.x = current_->pose.position.x + t * dx;
                p.pose.position.y = current_->pose.position.y + t * dy;
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, current_posture.getZ() + t * drotation);

                p.pose.orientation.w = q.getW();
                p.pose.orientation.x = q.getX();
                p.pose.orientation.y = q.getY();
                p.pose.orientation.z = q.getZ();

                new_path.poses.push_back(p);
            }

            publisher_->publish(new_path);
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(path_planner::PathPlanner)