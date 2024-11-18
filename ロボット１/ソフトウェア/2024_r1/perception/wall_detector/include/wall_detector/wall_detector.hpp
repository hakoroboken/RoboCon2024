#ifndef WALL_DETECTOR_HPP_
#define WALL_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

using std::placeholders::_1;

namespace wall_detector
{
    class WallDetector : public rclcpp::Node
    {
        public:
        explicit WallDetector(const rclcpp::NodeOptions option=rclcpp::NodeOptions());

        void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void PointCloud2Box(const sensor_msgs::msg::PointCloud pointcloud, visualization_msgs::msg::Marker *marker);

        private:
        int iter_num_param;
        double thre_param;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;

        visualization_msgs::msg::Marker marker;
    };
}

#endif