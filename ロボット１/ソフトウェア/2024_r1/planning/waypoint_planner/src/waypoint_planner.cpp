#include "waypoint_planner/waypoint_planner.hpp"

namespace waypoint_planner
{
    WayPointPlanner::WayPointPlanner(const rclcpp::NodeOptions option) : Node("WayPointPlanner", option)
    {
        target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot/pose/target", rclcpp::QoS(10));

        controller_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_id",
            rclcpp::QoS(10),
            std::bind(&WayPointPlanner::topic_callback, this, _1)
        );

        home = geometry_msgs::msg::PoseStamped();
        home.header.frame_id = "map";
        first_pose = geometry_msgs::msg::PoseStamped();
        first_pose.header.frame_id = "map";
        second_pose = geometry_msgs::msg::PoseStamped();
        second_pose.header.frame_id = "map";
        third_pose = geometry_msgs::msg::PoseStamped();
        third_pose.header.frame_id = "map";

        this->declare_parameter("home_x", 1.0);
        this->get_parameter("home_x", home.pose.position.x);
        this->declare_parameter("home_y", 1.0);
        this->get_parameter("home_y", home.pose.position.y);

        this->declare_parameter("first_x", 1.0);
        this->get_parameter("first_x", first_pose.pose.position.x);
        this->declare_parameter("first_y", 2.5);
        this->get_parameter("first_y", first_pose.pose.position.y);
        int f_yaw = 0;
        this->declare_parameter("first_yaw", 45);
        this->get_parameter("first_yaw", f_yaw);
        tf2::Quaternion q1;
        q1.setRPY(0.0, 0.0, (f_yaw * M_PI) / 180.0);
        first_pose.pose.orientation.w = q1.w();
        first_pose.pose.orientation.x = q1.x();
        first_pose.pose.orientation.y = q1.y();
        first_pose.pose.orientation.z = q1.z();

        this->declare_parameter("second_x", 1.0);
        this->get_parameter("second_x", second_pose.pose.position.x);
        this->declare_parameter("second_y", 2.5);
        this->get_parameter("second_y", second_pose.pose.position.y);
        int se_yaw = 0;
        this->declare_parameter("second_yaw", 0);
        this->get_parameter("second_yaw", se_yaw);
        tf2::Quaternion q2;
        q2.setRPY(0.0, 0.0, (f_yaw * M_PI) / 180.0);
        second_pose.pose.orientation.w = q2.w();
        second_pose.pose.orientation.x = q2.x();
        second_pose.pose.orientation.y = q2.y();
        second_pose.pose.orientation.z = q2.z();

        this->declare_parameter("third_x", 2.5);
        this->get_parameter("third_x", third_pose.pose.position.x);
        this->declare_parameter("third_y", 1.0);
        this->get_parameter("third_y", third_pose.pose.position.y);
        int th_yaw = 0;
        this->declare_parameter("third_yaw", 0);
        this->get_parameter("third_yaw", th_yaw);
        tf2::Quaternion q3;
        q3.setRPY(0.0, 0.0, (f_yaw * M_PI) / 180.0);
        third_pose.pose.orientation.w = q3.w();
        third_pose.pose.orientation.x = q3.x();
        third_pose.pose.orientation.y = q3.y();
        third_pose.pose.orientation.z = q3.z();

        RCLCPP_INFO(this->get_logger(), "Start WayPointPlanner");
        
    }

    void WayPointPlanner::topic_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if(msg->data == 1.0)
        {
            target_pose_publisher_->publish(home);
        }
        else if(msg->data == 2.0)
        {
            target_pose_publisher_->publish(first_pose);
        }
        else if(msg->data == 3.0)
        {
            target_pose_publisher_->publish(second_pose);
        }
        else if(msg->data == 4.0)
        {
            target_pose_publisher_->publish(third_pose);
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "NotCmd");
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_planner::WayPointPlanner)