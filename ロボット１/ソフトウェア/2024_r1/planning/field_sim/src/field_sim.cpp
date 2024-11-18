#include "field_sim/field_sim.hpp"

namespace field_sim
{
    FieldSim::FieldSim(const rclcpp::NodeOptions option) : Node("FieldSim", option)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/field", 0);

        timer_ = this->create_wall_timer(100ms, std::bind(&FieldSim::timer_callback, this));

        robot_pose = geometry_msgs::msg::Pose();

        this->declare_parameter("initial_pose_x", 0.0);
        this->get_parameter("initial_pose_x", robot_pose.position.x);
        this->declare_parameter("initial_pose_y", 0.0);
        this->get_parameter("initial_pose_y", robot_pose.position.y);
        this->declare_parameter("initial_pose_z", 0.0);
        this->get_parameter("initial_pose_z", robot_pose.position.z);

        auto line1 = visualization_msgs::msg::Marker();
        line1.header.frame_id = "map";
        line1.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line1.action = visualization_msgs::msg::Marker::ADD;
        line1.scale.x = 0.1;
        line1.color.r = 0.0;
        line1.color.g = 1.0;
        line1.color.b = 0.0;
        line1.color.a = 1.0;
        auto p1 = geometry_msgs::msg::Point();
        auto p2 = geometry_msgs::msg::Point();
        p2.x = 5;
        line1.points.push_back(p1);
        line1.points.push_back(p2);
        line1.id = 0;

        auto line2 = visualization_msgs::msg::Marker();
        line2.header.frame_id = "map";
        line2.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line2.action = visualization_msgs::msg::Marker::ADD;
        line2.scale.x = 0.1;
        line2.color.r = 0.0;
        line2.color.g = 1.0;
        line2.color.b = 0.0;
        line2.color.a = 1.0;
        auto p3 = geometry_msgs::msg::Point();
        p3.x = 5;
        p3.y = 3;
        line2.points.push_back(p2);
        line2.points.push_back(p3);
        line2.id = 1;

        auto line3 = visualization_msgs::msg::Marker();
        line3.header.frame_id = "map";
        line3.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line3.action = visualization_msgs::msg::Marker::ADD;
        line3.scale.x = 0.1;
        line3.color.r = 0.0;
        line3.color.g = 1.0;
        line3.color.b = 0.0;
        line3.color.a = 1.0;
        auto p4 = geometry_msgs::msg::Point();
        p4.x = 0;
        p4.y = 3;
        line3.points.push_back(p3);
        line3.points.push_back(p4);
        line3.id = 2;

        auto line4 = visualization_msgs::msg::Marker();
        line4.header.frame_id = "map";
        line4.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line4.action = visualization_msgs::msg::Marker::ADD;
        line4.scale.x = 0.1;
        line4.color.r = 0.0;
        line4.color.g = 1.0;
        line4.color.b = 0.0;
        line4.color.a = 1.0;
        line4.points.push_back(p4);
        line4.points.push_back(p1);
        line4.id = 3;

        ma.markers.push_back(line1);
        ma.markers.push_back(line2);
        ma.markers.push_back(line3);
        ma.markers.push_back(line4);
    }

    void FieldSim::timer_callback()
    {
        publisher_->publish(ma);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(field_sim::FieldSim)