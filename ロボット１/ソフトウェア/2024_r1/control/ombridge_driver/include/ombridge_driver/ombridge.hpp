#ifndef OMBRIDGE_HPP_
#define OMBRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <chrono>
#include <string>

#include "serial.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace ombridge_driver
{
    class OmBridgeDriver : public rclcpp::Node
    {
        public:
        explicit OmBridgeDriver(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());

        void move_1_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void move_2_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void bridge_1_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void bridge_2_callback(const std_msgs::msg::Float32::SharedPtr msg);

        void timer_callback();

        private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_move1;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_move2;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_bridge1;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_bridge2;

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<SerialHandler> serial;
        float move1_v, move2_v, bridge1_v, bridge2_v;
        std::string ser_path;
    };
}

#endif