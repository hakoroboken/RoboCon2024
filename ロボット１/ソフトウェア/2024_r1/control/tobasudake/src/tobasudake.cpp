#include "tobasudake/tobasudake.hpp"

namespace tobasudake
{
    TobasuDake::TobasuDake(const rclcpp::NodeOptions& option):Node("Tobasudake", option)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/to_pico", 0);

        x_sub_ = this->create_subscription<std_msgs::msg::Float32>("/out/x", 0, std::bind(&TobasuDake::x_topic_callback, this, _1));
        y_sub_ = this->create_subscription<std_msgs::msg::Float32>("/out/y", 0, std::bind(&TobasuDake::y_topic_callback, this, _1));
        rotation_sub_ = this->create_subscription<std_msgs::msg::Float32>("/out/rotation", 0, std::bind(&TobasuDake::rotation_topic_callback, this, _1));
        machine_sub = this->create_subscription<std_msgs::msg::Float32>("/out/machine", 0, std::bind(&TobasuDake::machine_callback, this, _1));

        timer_ = this->create_wall_timer(1ms, std::bind(&TobasuDake::timer_callback, this));

        prev_vector = geometry_msgs::msg::Twist();
        target_vector = geometry_msgs::msg::Twist();

        this->declare_parameter("smooth_gain", 0.1);
        this->get_parameter("smooth_gain", smooth_gain);
    }

    void TobasuDake::x_topic_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_vector.linear.x = msg->data;
        x_flag = true;
    }

    void TobasuDake::y_topic_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_vector.linear.y = msg->data;
        y_flag = true;
    }

    void TobasuDake::rotation_topic_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_vector.angular.z = msg->data;
        rotation_flag = true;
    }

    void TobasuDake::machine_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        m = msg->data;
    }

    void TobasuDake::timer_callback()
    {
        auto new_msg = std_msgs::msg::Int32MultiArray();

        auto vec = geometry_msgs::msg::Twist();
        vec.linear.x = target_vector.linear.x - prev_vector.linear.x;
        vec.linear.y = target_vector.linear.y - prev_vector.linear.y;
        vec.angular.z = target_vector.angular.z - prev_vector.angular.z;

        if(vec.linear.x > 0.0)
        {
            prev_vector.linear.x += smooth_gain;
        }
        else if(vec.linear.x < 0.0)
        {
            prev_vector.linear.x -= smooth_gain;
        }

        if(vec.linear.y > 0.0)
        {
            prev_vector.linear.y += smooth_gain;
        }
        else if(vec.linear.y < 0.0)
        {
            prev_vector.linear.y -= smooth_gain;
        }

        if(vec.angular.z > 0.0)
        {
            prev_vector.angular.z += smooth_gain;
        }
        else if(vec.angular.z < 0.0)
        {
            prev_vector.angular.z -= smooth_gain;
        }

        // prev_vector = target_vector;

        int32_t front = (prev_vector.linear.x + 0.8 * prev_vector.angular.z) * 255;
        int32_t rl = (-0.5*prev_vector.linear.x + 0.707106781 * prev_vector.linear.y + 0.8 * prev_vector.angular.z) * 255;
        int32_t rr = (-0.5*prev_vector.linear.x - 0.707106781 * prev_vector.linear.y + 0.8 * prev_vector.angular.z) * 255;
        int32_t m_v = m * 255;

        if(abs(front) > 255)
        {
            if(front > 0)
            {
                front = 255;
            }
            else
            {
                front = -255;
            }
        }
        if(abs(rl) > 255)
        {
            if(front > 0)
            {
                rl = 255;
            }
            else
            {
                rl = -255;
            }
        }
        if(abs(rr) > 255)
        {
            if(rr > 0)
            {
                rr = 255;
            }
            else
            {
                rr = -255;
            }
        }

        if(abs(front) < 50)front=0;
        if(abs(rl) < 50)rl=0;
        if(abs(rr) < 50)rr=0;

        new_msg.data.push_back(front);
        new_msg.data.push_back(rl);
        new_msg.data.push_back(rr);
        new_msg.data.push_back(m_v);

        publisher_->publish(new_msg);

        x_flag = false;
        y_flag = false;
        rotation_flag = false;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tobasudake::TobasuDake)