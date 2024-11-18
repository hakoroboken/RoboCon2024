#include "wall_detector/wall_detector.hpp"

namespace wall_detector
{
    WallDetector::WallDetector(const rclcpp::NodeOptions option) : Node("WallDetector", option)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/input", qos_settings, std::bind(&WallDetector::topic_callback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/wall", 0);

        this->declare_parameter("iter_num", 200);
        this->get_parameter("iter_num", iter_num_param);
        this->declare_parameter("threshold", 0.1);
        this->get_parameter("threshold", thre_param);


        RCLCPP_INFO(this->get_logger(), "Start WallDetector!!. iter_num:%d, threshold:%lf", iter_num_param, thre_param);
    }

    void WallDetector::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Start");
        auto input = sensor_msgs::msg::PointCloud();
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, input);

        auto filtered = sensor_msgs::msg::PointCloud();


        for(const auto& p : input.points)
        {
            RCLCPP_INFO(this->get_logger(), "p.z = %lf", p.z);
            if(p.z < 4.0)
            {
                filtered.points.push_back(p);
            }
        }

        filtered.header.frame_id = "camera_color_frame";
        publisher_->publish(filtered);
    }

    void WallDetector::PointCloud2Box(const sensor_msgs::msg::PointCloud pointcloud, visualization_msgs::msg::Marker *marker)
    {
        auto max_x_point = geometry_msgs::msg::Point();
        auto min_x_point = geometry_msgs::msg::Point();

        auto max_x = 0.0;
        auto min_x = 0.0;

        for(const auto& p : pointcloud.points)
        {
            if(p.x > max_x)
            {
                max_x = p.x;
                max_x_point.x = p.x;
                max_x_point.y = p.y;
                max_x_point.z = p.z;
            }

            if(min_x > p.x)
            {
                min_x = p.x;
                min_x_point.x = p.x;
                min_x_point.y = p.y;
                min_x_point.z = p.z;
            }
        }

        marker->header.frame_id = "camera_color_frame";
        marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker->action = visualization_msgs::msg::Marker::ADD;
        marker->scale.x = 0.1;
        marker->color.r = 0.0;
        marker->color.g = 0.0;
        marker->color.b = 1.0;
        marker->color.a = 1.0;
        marker->points.push_back(max_x_point);
        marker->points.push_back(min_x_point);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(wall_detector::WallDetector)