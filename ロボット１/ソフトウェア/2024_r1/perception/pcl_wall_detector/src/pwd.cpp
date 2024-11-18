#include "pcl_wall_detector/pwd.hpp"

namespace pcl_wall_detector
{
    PCLWallDetector::PCLWallDetector(const rclcpp::NodeOptions& option) : Node("PCL_WallDetector")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/input_pointcloud", 10,
            std::bind(&PCLWallDetector::pointcloud_callback, this, std::placeholders::_1));

        // Markerパブリッシャー
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/line_marker", 10);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_wall_detector::PCLWallDetector)