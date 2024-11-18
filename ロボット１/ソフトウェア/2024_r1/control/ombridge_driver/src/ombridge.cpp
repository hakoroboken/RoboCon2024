#include "ombridge_driver/ombridge.hpp"

namespace ombridge_driver
{
    OmBridgeDriver::OmBridgeDriver(const rclcpp:: NodeOptions& option) : Node("OmBridgeDriver", option)
    {
        sub_bridge1 = this->create_subscription<std_msgs::msg::Float32>("/bridge1", 0, std::bind(&OmBridgeDriver::bridge_1_callback, this, _1));
        sub_bridge2 = this->create_subscription<std_msgs::msg::Float32>("/bridge2", 0, std::bind(&OmBridgeDriver::bridge_2_callback, this, _1));
        sub_move1 = this->create_subscription<std_msgs::msg::Float32>("/move1", 0, std::bind(&OmBridgeDriver::move_1_callback, this, _1));
        sub_move2 = this->create_subscription<std_msgs::msg::Float32>("/move2", 0, std::bind(&OmBridgeDriver::move_2_callback, this, _1));

        bridge1_v = 0.0;
        bridge2_v = 0.0;
        move1_v = 0.0;
        move2_v = 0.0;

        timer_ = this->create_wall_timer(1ms, std::bind(&OmBridgeDriver::timer_callback, this));

        this->declare_parameter("path", "/dev/ttyUSB0");
        this->get_parameter("path", ser_path);

        serial = std::make_shared<SerialHandler>();
        serial->OpenPort(ser_path);
    }

    void OmBridgeDriver::bridge_1_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        bridge1_v = msg->data;
    }
    void OmBridgeDriver::bridge_2_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        bridge2_v = msg->data;
    }
    void OmBridgeDriver::move_1_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        move1_v = msg->data;
    }
    void OmBridgeDriver::move_2_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        move2_v = msg->data;
    }

    void OmBridgeDriver::timer_callback()
    {
        auto m1_i = 20 + (int)(move1_v * 10.0);
        auto m2_i = 20 + (int)(move2_v * 10.0);
        auto m3_i = 20 + (int)(bridge1_v * 10.0);
        auto m4_i = 20 + (int)(bridge2_v * 10.0);
        auto str = std::to_string(m1_i) + ',' + std::to_string(m2_i) + ',' + std::to_string(m3_i) + ',' + std::to_string(m4_i) + 'e';

        RCLCPP_INFO(this->get_logger(), "Get Cmd : %s", str.c_str());

        serial->WritePort(str);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ombridge_driver::OmBridgeDriver)