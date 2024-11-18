#include "visual_odom_ros2/vo_ros2.hpp"

namespace visual_odom_ros2
{
    VisualOdomROS2::VisualOdomROS2(const rclcpp::NodeOptions option) : Node("VisualOdomROS2", option)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        rgb_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image/rgb",
            qos_settings,
            std::bind(&VisualOdomROS2::rgb_callback, this, _1));

        depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image/depth",
            qos_settings,
            std::bind(&VisualOdomROS2::depth_callback, this, _1));

        rpy_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/rpy",
            0,
            std::bind(&VisualOdomROS2::rpy_callback, this, _1)
        );

        current_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot/pose/current", 0);

        vo = std::make_shared<VisualOdometer>();

        timer_ = this->create_wall_timer(1ms, std::bind(&VisualOdomROS2::timer_callback, this));

        this->declare_parameter("frame_id", "map");
        this->get_parameter("frame_id", frame_id_param);

        rpy = geometry_msgs::msg::Vector3();

        rgb_flag = false;
        depth_flag = false;
    }

    void VisualOdomROS2::rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "RGB : %d", msg->header.stamp.nanosec);
        if(!rgb_flag)
        {
            vo->setRGBImage(msg);
            // RCLCPP_INFO(this->get_logger(), "Debug rgb %s", msg->encoding.c_str());
            rgb_flag = true;
        }
    }

    void VisualOdomROS2::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Depth : %d", msg->header.stamp.nanosec);
        if(!depth_flag)
        {
            vo->setDepthImage(msg);
            // RCLCPP_INFO(this->get_logger(), "Debug depth %s", msg->encoding.c_str());
            depth_flag = true;
        }
    }

    void VisualOdomROS2::rpy_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        rpy.x = msg->x;
        rpy.y = msg->y;
        rpy.z = msg->z;
    }

    void VisualOdomROS2::timer_callback()
    {
        try
        {
            if(rgb_flag && depth_flag)
            {
                // RCLCPP_INFO(this->get_logger(), "Start VO");
                if(vo->prev_rgb_image_.empty())
                {
                    // RCLCPP_INFO(this->get_logger(), "Copy current to previeous");
                    vo->prev_rgb_image_ = vo->current_rgb_image_.clone();
                    rgb_flag = false;
                    depth_flag = false;
                    return;
                }

                auto orb = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.001f);
                std::vector<cv::KeyPoint> keypoints_prev, keypoints_current;
                cv::Mat desc_prev, desc_curr;
                // RCLCPP_INFO(this->get_logger(), "Start detect");
                orb->detect(vo->prev_rgb_image_, keypoints_prev, cv::noArray());
                orb->detect(vo->current_rgb_image_, keypoints_current, cv::noArray());
                // RCLCPP_INFO(this->get_logger(), "Start compute");
                orb->compute(vo->prev_rgb_image_, keypoints_prev, desc_prev);
                orb->compute(vo->current_rgb_image_, keypoints_current, desc_curr);

                // RCLCPP_INFO(this->get_logger(), "Detect and compute");

                Matches matches;
                cv::BFMatcher matcher(cv::NORM_HAMMING, true);
                matcher.match(desc_prev, desc_curr, matches);

                // RCLCPP_INFO(this->get_logger(), "Matched");

                std::vector<cv::Point3f> points3D;
                std::vector<cv::Point2f> points2D;
                for (const auto& match : matches) {
                    cv::Point2f pt_prev = keypoints_prev[match.queryIdx].pt;
                    cv::Point2f pt_curr = keypoints_current[match.trainIdx].pt;

                    // 深度画像を用いて3Dポイントを再構築
                    auto d = vo->current_depth_image_.at<uint16_t>(pt_prev.y, pt_prev.x);
                    float depth = d / 1000.0f;
                    if (depth > 0) { // 有効な深度値か確認
                        float x = (pt_prev.x - vo->K_.at<double>(0, 2)) * depth / vo->K_.at<double>(0, 0);
                        float y = (pt_prev.y - vo->K_.at<double>(1, 2)) * depth / vo->K_.at<double>(1, 1);
                        float z = depth;
                        points3D.push_back(cv::Point3f(x, y, z));
                        points2D.push_back(pt_curr);
                    }
                }

                tf2::Quaternion q;
                bool fl = false;
                if(!points3D.empty() && !points2D.empty())
                {
                    cv::Mat R, t, inliers, rvec;
                    fl = cv::solvePnPRansac(points3D, points2D, vo->K_, cv::Mat(), rvec, t, true, 400, 8.0, 0.99, inliers);
                
                    cv::Rodrigues(rvec, R);

                    
                    q.setRPY(rpy.x, rpy.y, rpy.z);
                    tf2::Matrix3x3 mat(q);
                    cv::Mat cvRotationMatrix = cv::Mat::zeros(3, 3, CV_64F); // CV_64Fでdouble型の3x3行列
                    for (int i = 0; i < 3; ++i)
                    {
                        for (int j = 0; j < 3; ++j)
                        {
                            cvRotationMatrix.at<double>(i, j) = mat[i][j];
                        }
                    }
                    cv::Mat R_inv = cvRotationMatrix.t();
                    cv::Mat t_inv = -R_inv * t;
                    vo->t_f_ = R_inv * vo->t_f_ + t_inv;
                    vo->R_f_ = R_inv * vo->R_f_;
                    tf2::Matrix3x3 m;
                    
                }

                vo->prev_rgb_image_ = vo->current_rgb_image_.clone();

                RCLCPP_INFO(this->get_logger(), "Processed visual odometry., %d", fl);
                
                auto result = vo->getOdom();

                auto pose = geometry_msgs::msg::PoseStamped();

                pose.header.frame_id = frame_id_param;
                pose.pose.position.x = result.x;
                pose.pose.position.y = result.y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = q.w();
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                RCLCPP_INFO(this->get_logger(), "x:%lf, y:%lf, z:%lf", result.x, result.y, result.z);

                current_publisher_->publish(pose);

                rgb_flag = false;
                depth_flag = false;
            }
        }
        catch(cv_bridge::Exception e)
        {
            RCLCPP_ERROR(this->get_logger(), "ERR %s", e.what());
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(visual_odom_ros2::VisualOdomROS2)