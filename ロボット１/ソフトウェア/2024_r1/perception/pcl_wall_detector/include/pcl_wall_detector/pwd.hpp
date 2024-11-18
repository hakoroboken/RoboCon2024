#ifndef PWD_HPP_
#define PWD_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace pcl_wall_detector
{
    class PCLWallDetector : public rclcpp::Node
    {
        public:
        explicit PCLWallDetector(const rclcpp::NodeOptions& option=rclcpp::NodeOptions());

        private:
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg, *cloud2);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for(const auto& p : cloud2->points)
            {
                auto new_p = pcl::PointXYZ();
                if(-1.0 * p.y < 0.0 );
                {
                    new_p.x = p.z;
                    new_p.y = p.x;
                    new_p.z = -p.y;

                    cloud->points.push_back(new_p);
                }
            }

            // ダウンサンプリング
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(cloud);
            voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
            voxel_filter.filter(*cloud_filtered);

            // 平面検出
            pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
            pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
            plane_seg.setOptimizeCoefficients(true);
            plane_seg.setModelType(pcl::SACMODEL_PLANE);
            plane_seg.setMethodType(pcl::SAC_RANSAC);
            plane_seg.setDistanceThreshold(0.01);  // 適宜設定
            plane_seg.setInputCloud(cloud_filtered);
            plane_seg.segment(*plane_inliers, *plane_coefficients);

            if (plane_inliers->indices.empty()) {
                RCLCPP_INFO(this->get_logger(), "平面が見つかりませんでした");
                return;
            }

            // 平面のインライア点を除去
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_plane(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(plane_inliers);
            extract.setNegative(true);  // trueで平面以外を抽出
            extract.filter(*cloud_without_plane);

            // 直線検出
            pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients());
            pcl::SACSegmentation<pcl::PointXYZ> line_seg;
            line_seg.setOptimizeCoefficients(true);
            line_seg.setModelType(pcl::SACMODEL_LINE);
            line_seg.setMethodType(pcl::SAC_RANSAC);
            line_seg.setDistanceThreshold(0.1);
            line_seg.setInputCloud(cloud_without_plane);
            line_seg.segment(*line_inliers, *line_coefficients);

            if (line_inliers->indices.empty()) {
                RCLCPP_INFO(this->get_logger(), "直線が見つかりませんでした");
                return;
            }

            // 直線の開始点と方向ベクトルからエンドポイントを計算
            geometry_msgs::msg::Point start_point;
            start_point.x = line_coefficients->values[0];
            start_point.y = line_coefficients->values[1];
            start_point.z = line_coefficients->values[2];

            geometry_msgs::msg::Point end_point;
            end_point.x = start_point.x + line_coefficients->values[3];
            end_point.y = start_point.y + line_coefficients->values[4];
            end_point.z = start_point.z + line_coefficients->values[5];

            auto y = (start_point.x + end_point.x) / 2.0;
            RCLCPP_INFO(this->get_logger(), "Detected : %lf", y);

            // Markerメッセージの設定
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "camera_color_frame";
            line_marker.header.stamp = this->get_clock()->now();
            line_marker.ns = "line_detection";
            line_marker.id = 0;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.05;
            line_marker.color.a = 1.0;
            line_marker.color.r = 1.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 0.0;

            // Markerの点を追加
            line_marker.points.push_back(start_point);
            line_marker.points.push_back(end_point);

            // Markerをパブリッシュ
            marker_publisher_->publish(line_marker);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    };
}

#endif