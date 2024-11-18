#include "dualsense_ros2/dualsense_ros2_node.hpp"

namespace dualsense_ros2
{
    DualSenseROS2::DualSenseROS2(const rclcpp::NodeOptions & node_options) : rclcpp::Node("DualSenseROS2", node_options)
    {
        linear_x_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/controller/x", 0);
        linear_y_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/controller/y", 0);
        angular_z_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/controller/rotation", 0);

        f_publisher_1 = this->create_publisher<std_msgs::msg::Float32>("/f1", 0);
        f_publisher_2 = this->create_publisher<std_msgs::msg::Float32>("/f2", 0);
        f_publisher_3 = this->create_publisher<std_msgs::msg::Float32>("/f3", 0);
        f_publisher_4 = this->create_publisher<std_msgs::msg::Float32>("/f4", 0);
        f_publisher_5 = this->create_publisher<std_msgs::msg::Float32>("/f5", 0);
        f_publisher_9 = this->create_publisher<std_msgs::msg::Float32>("/f9", 0);
        f_publisher_6 = this->create_publisher<std_msgs::msg::Float32>("/f6", 0);
        f_publisher_7 = this->create_publisher<std_msgs::msg::Float32>("/f7", 0);
        f_publisher_8 = this->create_publisher<std_msgs::msg::Float32>("/f8", 0);

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 0, std::bind(&DualSenseROS2::timer_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "DualSenseROS2 intialize OK!!");
    }

    void DualSenseROS2::timer_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto x_msg = std_msgs::msg::Float32();
        auto y_msg = std_msgs::msg::Float32();
        auto rotation_msg = std_msgs::msg::Float32();

        x_msg.data = -1.0* msg->axes[0];
        y_msg.data = msg->axes[1];
        rotation_msg.data = -1.0* msg->axes[3];

        linear_x_publisher_->publish(x_msg);
        linear_y_publisher_->publish(y_msg);
        angular_z_publisher_->publish(rotation_msg);

        auto f1_msg = std_msgs::msg::Float32();
        auto f2_msg = std_msgs::msg::Float32();
        auto f3_msg = std_msgs::msg::Float32();
        auto f4_msg = std_msgs::msg::Float32();
        auto f5_msg = std_msgs::msg::Float32();
        auto f6_msg = std_msgs::msg::Float32();
        auto f7_msg = std_msgs::msg::Float32();
        auto f8_msg = std_msgs::msg::Float32();
        auto f9s_msg = std_msgs::msg::Float32();

        // left right
        f1_msg.data = -1.0 * msg->axes[7];

        // R1 R2
        f2_msg.data = msg->buttons[5] - msg->buttons[7];

        // up down
        f3_msg.data = -1.0 * msg->axes[6];

        f4_msg.data = msg->buttons[1] - msg->buttons[2];
        f5_msg.data = msg->buttons[0] - msg->buttons[3];

        f6_msg.data = msg->buttons[2];
        f7_msg.data = msg->buttons[3];

        f_publisher_1->publish(f1_msg);
        f_publisher_2->publish(f2_msg);
        f_publisher_3->publish(f3_msg);
        f_publisher_4->publish(f4_msg);
        f_publisher_5->publish(f5_msg);
        f_publisher_6->publish(f6_msg);
        f_publisher_7->publish(f7_msg);
        f_publisher_8->publish(f8_msg);
        f_publisher_9->publish(f9s_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dualsense_ros2::DualSenseROS2)