#include "visual_odom_ros2/vo.hpp"

namespace visual_odom_ros2
{
    VisualOdometer::VisualOdometer()
    {
        K_ = (cv::Mat_<double>(3, 3) << 617.0, 0, 320.5, 0, 617.0, 240.5, 0, 0, 1);
        R_f_ = cv::Mat::eye(3, 3, CV_64F);
        t_f_ = (cv::Mat_<double>(3, 1) << 1.0, 1.0, 0.0);
    }

    void VisualOdometer::setRGBImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            auto color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
            cv::cvtColor(color, current_rgb_image_, cv::COLOR_RGB2GRAY);
        }
        catch(cv_bridge::Exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void VisualOdometer::setDepthImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            current_depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        }
        catch(cv_bridge::Exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    bool VisualOdometer::process()
    {
        if(prev_rgb_image_.empty())
        {
            prev_rgb_image_ = current_rgb_image_.clone();
            return false;
        }

        cv::Ptr<cv::ORB> orb = cv::ORB::create();

        std::vector<cv::KeyPoint> keypoints_prev, keypoints_current;
        
        cv::Mat desc_prev, desc_curr;
        orb->detectAndCompute(prev_rgb_image_, cv::noArray(), keypoints_prev, desc_prev);
        orb->detectAndCompute(current_rgb_image_, cv::noArray(), keypoints_current, desc_curr);

        Matches matches;
        cv::BFMatcher matcher(cv::NORM_HAMMING, true);
        matcher.match(desc_prev, desc_curr, matches);

        cv3PointCloud points3prev;
        cv3PointCloud points3current;

        for(const auto& match : matches)
        {
            auto pt_prev = keypoints_prev[match.queryIdx].pt;
            auto pt_curr = keypoints_current[match.trainIdx].pt;

            float depth = current_depth_image_.at<uint16_t>(static_cast<int>(pt_prev.y), static_cast<int>(pt_prev.x)) / 1000.0;
            

            if(depth > 0)
            {
                auto x = (pt_prev.x - K_.at<double>(0, 2)) * depth / K_.at<double>(0, 0);
                auto y = (pt_prev.y - K_.at<double>(1, 2)) * depth / K_.at<double>(1, 1);
                auto z = depth;

                auto x_ = (pt_curr.x - K_.at<double>(0, 2)) * depth / K_.at<double>(0, 0);
                auto y_ = (pt_curr.y - K_.at<double>(1, 2)) * depth / K_.at<double>(1, 1);
                auto z_ = depth;

                points3prev.push_back(cv::Point3f(x, y, z));
                points3current.push_back(cv::Point3f(x_, y_, z_));
            }
        }

        bool fl = false;
        if(!points3current.empty() && !points3prev.empty())
        {
            cv::Mat R, t, inliers;
            R = cv::Mat::eye(3, 3, CV_64F);
            t = cv::Mat::zeros(3, 1, CV_64F);
            fl = cv::solvePnPRansac(points3prev, points3current, K_, cv::noArray(), R, t, false, 100, 1.0, 0.99, inliers);
            

            R_f_ = R * R_f_;
            t_f_ = t_f_ + R_f_ * t;
        }

        prev_rgb_image_ = current_rgb_image_.clone();
        return fl;
    }

    geometry_msgs::msg::Vector3 VisualOdometer::getOdom()
    {
        auto vec = geometry_msgs::msg::Vector3();

        vec.x = t_f_.at<double>(0);
        vec.y = t_f_.at<double>(1);
        vec.z = t_f_.at<double>(2);

        return vec;
    }
}