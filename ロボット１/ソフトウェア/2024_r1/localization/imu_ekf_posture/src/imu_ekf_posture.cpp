#include "imu_ekf_posture.hpp"

namespace imu_ekf_posture
{
    ImuEkfPosture::ImuEkfPosture(const rclcpp::NodeOptions & node_options): rclcpp::Node("imu_ekf_posture_node", node_options)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            qos_settings,
            std::bind(&ImuEkfPosture::imu_callback, this, _1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        rpy_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("rpy", 0);

        this->declare_parameter("enable_position", false);
        this->get_parameter("enable_position", enable_position_);

        this->declare_parameter("frame_id", "odom");
        this->get_parameter("frame_id", frame_id_);

        this->declare_parameter("child_frame_id", "imu");
        this->get_parameter("child_frame_id", child_id_);

        imu_flag_ = false;

        ekf6_ = std::make_shared<Axis6EKF>();
        prev_input_ = Vector3();
        prev_output_ = Vector3();
        RCLCPP_INFO(this->get_logger(), "Initialized EKF");

        timer_ = this->create_wall_timer(1ms, std::bind(&ImuEkfPosture::axis6_callback, this));

        RCLCPP_INFO(this->get_logger(), "Start MotiOdom delta_time: 10ms");

        prev_z = 0.0;
    }

    void ImuEkfPosture::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_flag_ = true;
        get_imu_ = msg;
    }


    void ImuEkfPosture::axis6_callback()
    {
        if(imu_flag_)
        {
            
            auto linear_accel = Vector3(
                get_imu_->linear_acceleration.x,
                get_imu_->linear_acceleration.z,
                -1.0 * get_imu_->linear_acceleration.y);

            auto angular_velocity = Vector3(
                get_imu_->angular_velocity.x,
                get_imu_->angular_velocity.z,
                -1.0 * get_imu_->angular_velocity.y);

            auto input_matrix = Vector3(
                angular_velocity.x*0.001,
                angular_velocity.y*0.001,
                angular_velocity.z*0.001);


            auto estimated = ekf6_->run_ekf6(input_matrix, linear_accel);
            
            // if(fabs(estimated.z) < 0.01)
            // {
            //     estimated.z = 0.0;
            //     ekf6_->est.z = 0.0;
            // }

            geometry_msgs::msg::TransformStamped t;
            geometry_msgs::msg::Vector3 rpy_msg;

            t.header.frame_id = frame_id_;
            t.header.stamp = this->get_clock()->now();
            t.child_frame_id = child_id_;

            tf2::Quaternion q;
            q.setRPY(estimated.x, estimated.y/ 4.0, estimated.z);
            // RCLCPP_INFO(this->get_logger(), "%lf", estimated.z);
            t.transform.rotation.w = q.w();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();

            tf_broadcaster_->sendTransform(t);
            rpy_msg.x = estimated.x;
            rpy_msg.y = estimated.y;
            rpy_msg.z = estimated.z;
            rpy_publisher_->publish(rpy_msg);

            prev_z = estimated.z;
        }
    }

    Vector3 ImuEkfPosture::remove_gravity(Vector3 linear_accel, Vector3 euler, float gravity)
    {
        auto rm = rotation_from_euler(euler);

        auto g = Vector3(0.0, 0.0, gravity);

        auto removed = multiply(rm, g);

        auto g_removed = Vector3(
            removed.x + linear_accel.x,
            removed.y + linear_accel.y,
            removed.z - linear_accel.z);

        return g_removed;
    }

    Vector3 ImuEkfPosture::noise_filter(Vector3 value, float alpha)
    {
        Vector3 output;
        output.x = alpha * (prev_output_.x + value.x - prev_output_.x);
        output.y = alpha * (prev_output_.y + value.y - prev_output_.y);
        output.z = alpha * (prev_output_.z + value.z - prev_output_.z);

        prev_output_ = output;
        prev_input_ = value;

        return output;

    }

    float ImuEkfPosture::to_radian(float degree)
    {
        auto pi = acos(-1.0);


        return (degree*pi)/180.0;
    }

    Matrix3x3 ImuEkfPosture::rotation_from_euler(Vector3 euler)
    {
        auto sin_x = sin(euler.x);
        auto sin_y = sin(euler.y);
        auto sin_z = sin(euler.z);
        auto cos_x = cos(euler.x);
        auto cos_y = cos(euler.y);
        auto cos_z = cos(euler.z);
        return Matrix3x3(
            cos_y*cos_z, -1.0*cos_y*sin_z, sin_y,
            sin_x*sin_y*cos_z+cos_x*sin_z, -1.0*sin_x*sin_y*sin_z + cos_x*cos_z, -1.0*sin_x*cos_y,
            -1.0*cos_x*sin_y*cos_z + sin_x*sin_z, cos_x*sin_y*sin_z + sin_x*cos_z, cos_x*cos_y
        );
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_ekf_posture::ImuEkfPosture)