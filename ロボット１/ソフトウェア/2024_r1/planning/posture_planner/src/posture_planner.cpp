#include "posture_planner/posture_planner.hpp"

namespace posture_planner
{
    PosturePlanner::PosturePlanner(const rclcpp::NodeOptions option) : Node("PosturePlanner", option)
    {
        current_flag = false;
        target_flag = false;
        current_yaw = 0.0;
        target_yaw = 0.0;

        current_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/current", 
            0, 
            std::bind(&PosturePlanner::current_sub_callback, this, _1));

        target_sub = this->create_subscription<std_msgs::msg::Float32>(
            "/target",
            0,
            std::bind(&PosturePlanner::target_sub_callback, this, _1));

        angular_vel_publisher = this->create_publisher<std_msgs::msg::Float32>("/angular", 0);

        this->declare_parameter("threshold", 0.3);
        this->get_parameter("threshold", threshold);
        this->declare_parameter("enable_reverse", false);
        this->get_parameter("enable_reverse", enable_reverse);

        timer_ = this->create_wall_timer(1ms, std::bind(&PosturePlanner::timer_callback, this));
    }

    void PosturePlanner::current_sub_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        current_yaw = msg->z;
        current_flag = true;
    }

    void PosturePlanner::target_sub_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if(msg->data == 1.0)
        {
            if(enable_reverse)
            {
                target_yaw = M_PI / -2.0;
            }
            else
            {
                target_yaw = M_PI / 2.0;
            }
        }
        else if(msg->data == -1.0)
        {
            target_yaw = 0.0;
        }
        
        target_flag = true;
    }

    void PosturePlanner::timer_callback()
    {
        if(current_flag && target_flag)
        {
            auto yaw_error = target_yaw - current_yaw;

            auto msg = std_msgs::msg::Float32();

            msg.data = -1.2 * yaw_error;

            angular_vel_publisher->publish(msg);

            current_flag = false;
            target_flag = false;

            RCLCPP_INFO(this->get_logger(), "target:%lf, current:%lf", target_yaw, current_yaw);
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(posture_planner::PosturePlanner)