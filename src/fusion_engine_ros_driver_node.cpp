//  Copyright 2023 Siddharth Saha
/**
 * @brief Source file containing implementation of functions in Fusion Engine ROS Driver node header file
*/

#include "fusion_engine_ros_driver/fusion_engine_ros_driver_node.hpp"
#include "fusion_engine_ros_driver/fusion_engine_ros_driver_utils.hpp"

#include "point_one/fusion_engine/messages/ros.h"
namespace point_one
{
namespace fusion_engine
{
namespace ros_driver
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

    framer_ = std::make_unique<point_one::fusion_engine::parsers::FusionEngineFramer>(capacity);
    framer_->SetMessageCallback(std::bind(&FusionEngineRosDriverNode::PublishMessage, this, std::placeholders::_1, std::placeholders::_2));
}

void FusionEngineRosDriverNode::UdpCallback(const udp_msgs::msg::UdpPacket::SharedPtr udp_msg)
{
    auto udp_data = udp_msg->data;
    msg_received_time_ = this->now();
    point_one::fusion_engine::ros_utils::SendDataToFramer(udp_data, framer_);
}

void FusionEngineRosDriverNode::SerialCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr serial_msg)
{
    auto serial_data = serial_msg->data;
    msg_received_time_ = this->now();
    point_one::fusion_engine::ros_utils::SendDataToFramer(serial_data, framer_);
}

void FusionEngineRosDriverNode::PublishMessage(const point_one::fusion_engine::messages::MessageHeader& header, const void* payload_in)
{
    auto payload = static_cast<const uint8_t*>(payload_in);

    if (header.message_type == point_one::fusion_engine::messages::MessageType::ROS_GPS_FIX) {
        auto& contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::GPSFixMessage*>(payload);
        gps_msgs::msg::GPSFix::SharedPtr gps_fix_msg = std::make_shared<gps_msgs::msg::GPSFix>();
        sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
        point_one::fusion_engine::ros_utils::toGPSFix(contents, gps_fix_msg);
        point_one::fusion_engine::ros_utils::toNavSatFix(contents, nav_sat_fix_msg);
        gps_fix_msg->header.frame_id = frame_id_;
        gps_fix_msg->header.stamp = msg_received_time_;
        nav_sat_fix_msg->header = gps_fix_msg->header;
        gps_fix_publisher_->publish(*gps_fix_msg);
        nav_sat_fix_publisher_->publish(*nav_sat_fix_msg);
    } else if (header.message_type == point_one::fusion_engine::messages::MessageType::ROS_IMU) {
        auto& contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::IMUMessage*>(payload);
        sensor_msgs::msg::Imu::SharedPtr imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        point_one::fusion_engine::ros_utils::toImu(contents, imu_msg);
        imu_msg->header.frame_id = frame_id_;
        imu_msg->header.stamp = msg_received_time_;
        imu_publisher_->publish(*imu_msg);
    } else if (header.message_type == point_one::fusion_engine::messages::MessageType::ROS_POSE) {
        auto& contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::PoseMessage*>(payload);
        geometry_msgs::msg::PoseStamped::SharedPtr pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        point_one::fusion_engine::ros_utils::toP1ROSPose(contents, pose_msg);
        pose_msg->header.frame_id = frame_id_;
        pose_msg->header.stamp = msg_received_time_;
        pose_publisher_->publish(*pose_msg);
    }
}

}  // namespace ros_driver
}  // namespace fusion_engine
}  // namespace point_one

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(point_one::fusion_engine::ros_driver::FusionEngineRosDriverNode)