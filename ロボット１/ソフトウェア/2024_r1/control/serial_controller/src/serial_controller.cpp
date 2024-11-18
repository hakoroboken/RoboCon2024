#include "serial_controller/serial_controller.hpp"

namespace serial_controller
{
    SerialController::SerialController(const rclcpp::NodeOptions option) : Node("SerialController", option)
    {
        sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/to_pico", 
            0, 
            std::bind(&SerialController::topic_callback, this, _1));

        timer_ = this->create_wall_timer(100ms, std::bind(&SerialController::timer_callback, this));

        this->declare_parameter("port_path", "/dev/ttyACM0");
        this->get_parameter("port_path", port_path_param);

        serial = std::make_shared<SerialHandler>();

        auto er = serial->OpenPort(port_path_param);

        if(er)
        {
            RCLCPP_INFO(this->get_logger(), "Start SerialController");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial : %s", port_path_param.c_str());
            serial->ClosePort();
        }

        str_flag = false;
    }

    void SerialController::topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        std::string str = std::to_string(msg->data[0]+300) + ',' + std::to_string(msg->data[1]+300) + ',' + std::to_string(msg->data[2]+300) + ',' + std::to_string(msg->data[3]+300) + 'e';

        tx = str;

        str_flag = true;
    }

    void SerialController::timer_callback()
    {
        if(str_flag)
        {
            bool err = serial->WritePort(tx);

            if(err)
            {
                // RCLCPP_INFO(this->get_logger(), "Write %s", tx.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write serial : %s", tx.c_str());
                serial->ClosePort();
            }

            str_flag = false;
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(serial_controller::SerialController)