//  Copyright 2023 Siddharth Saha
/**
 * @brief Source file containing implementation of functions in Fusion Engine ROS Driver header file
*/

#include "fusion_engine_ros_driver/fusion_engine_ros_driver_node.hpp"

namespace pointone
{
namespace fusion_engine
{
namespace ros
{
FusionEngineRosDriverNode::FusionEngineRosDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fusion_engine_ros_driver_node", options)
{
    frame_id_ = this->declare_parameter<std::string>("frame_id");
    connection_type_ = this->declare_parameter<std::string>("connection_type");
    size_t capacity = this->declare_parameter<int>("capacity_bytes");

    udp_subscriber_ = this->create_subscription<udp_msgs::msg::UdpPacket>("fusion_engine_udp_packets", rclcpp::QoS{100}, std::bind(&FusionEngineRosDriverNode::UdpCallback, this, std::placeholders::_1));
    serial_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>("fusion_engine_serial_packets", rclcpp::QoS{100}, std::bind(&FusionEngineRosDriverNode::SerialCallback, this, std::placeholders::_1));

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", rclcpp::SensorDataQoS());
    gps_fix_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("gps_fix", rclcpp::SensorDataQoS());
    nav_sat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix", rclcpp::SensorDataQoS());
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());

    // framer_ = std::make_unique<point_one::fusion_engine::parsers::FusionEngineFramer>(capacity);
    // framer_->SetMessageCallback(PublishMessage);
}
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointone::fusion_engine::ros::FusionEngineRosDriverNode)