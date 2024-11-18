#ifndef VO_HPP_
#define VO_HPP_

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <eigen3/Eigen/Dense>

namespace visual_odom_ros2
{
    typedef std::vector<cv::DMatch> Matches;
    typedef std::vector<cv::Point3f> cv3PointCloud;
    typedef std::vector<cv::Point2f> cv2PointCloud;

    class VisualOdometer
    {
        public:
        VisualOdometer();
        
        void setRGBImage(const sensor_msgs::msg::Image::SharedPtr msg);
        void setDepthImage(const sensor_msgs::msg::Image::SharedPtr msg);
        bool process();
        geometry_msgs::msg::Vector3 getOdom();
        
        cv::Mat prev_rgb_image_, current_rgb_image_;
        cv::Mat current_depth_image_;
        cv::Mat K_;
        cv::Mat R_f_;
        cv::Mat t_f_;
    };
}

#endif