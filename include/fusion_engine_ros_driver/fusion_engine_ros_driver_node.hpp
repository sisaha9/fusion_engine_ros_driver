//  Copyright 2023 Siddharth Saha
/**
 * @brief Header file containing definition of node class
*/

#ifndef FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_NODE_HPP_
#define FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

#include "point_one/fusion_engine/messages/core.h"
#include "point_one/fusion_engine/parsers/fusion_engine_framer.h"

namespace point_one
{
namespace fusion_engine
{
namespace ros_driver
{
class FusionEngineRosDriverNode : public rclcpp::Node
{
public:
    explicit FusionEngineRosDriverNode(const rclcpp::NodeOptions & options);
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr udp_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_subscriber_;

    void UdpCallback(const udp_msgs::msg::UdpPacket::SharedPtr udp_msg);
    void SerialCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr serial_msg);
    void PublishMessage(const point_one::fusion_engine::messages::MessageHeader& header, const void* payload_in);

    std::string connection_type_{""};
    std::string frame_id_{""};

    std::unique_ptr<point_one::fusion_engine::parsers::FusionEngineFramer> framer_;

    rclcpp::Time msg_received_time_;
};
}  // namespace ros_driver
}  // namespace fusion_engine
}  // namespace point_one

#endif  // FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_NODE_HPP_