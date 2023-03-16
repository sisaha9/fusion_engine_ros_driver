//  Copyright 2023 Siddharth Saha
/**
 * @brief Header file containing definition of node class
*/

#ifndef FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_NODE_HPP_
#define FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_NODE_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "point_one/fusion_engine/messages/core.h"
#include "point_one/fusion_engine/parsers/fusion_engine_framer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

namespace point_one {
namespace fusion_engine {
namespace ros_driver {
/**
 * @brief ROS 2 Node that acts as a Fusion Engine Client and publishes incoming
 * data
 * 
 * This class has subscribers to UDP Message packet and a Standard Uint8 array
 * message. These are published by the UDP driver and Serial driver that lies in
 * ROS 2 transport drivers repo
 * (https://github.com/ros-drivers/transport_drivers)
 * 
 * It then publishes a PoseStamped, GPSFix, NavSatFix and IMU message that are
 * standard ROS 2 messages published by GNSS/INS devices
 * 
 * Example usage:
 * ```cpp
 * #include <rclcpp_components/register_node_macro.hpp>
 * RCLCPP_COMPONENTS_REGISTER_NODE(
 *   point_one::fusion_engine::ros_driver::FusionEngineRosDriverNode
 * )
 * ```
 * It is highly recommended to use ROS 2 components as displayed above.
 * Benefits explained here
 * https://docs.ros.org/en/rolling/Concepts/About-Composition.html
 */
class FusionEngineRosDriverNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Fusion Engine Ros Driver Node object
   * 
   * @param options Options for the node. Defined here: 
   * https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1NodeOptions.html
   */
  explicit FusionEngineRosDriverNode(const rclcpp::NodeOptions& options);

 private:
  /**
   * @brief ROS 2 Pose Publisher
   * 
   */
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  /**
   * @brief ROS 2 GPS Fix Publisher
   * 
   */
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_publisher_;
  /**
   * @brief ROS 2 Nav Sat Fix Publisher
   * 
   */
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      nav_sat_fix_publisher_;
  /**
   * @brief ROS 2 IMU Publisher
   * 
   */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  /**
   * @brief ROS 2 UDP Subscriber
   * 
   * This is where UDP packets will come in
   */
  rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr udp_subscriber_;

  /**
   * @brief ROS 2 Uint8 Array Subscriber
   * 
   * This is where Serial packets will come in
   */
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
      serial_subscriber_;

  /**
   * @brief Function called when Udp message comes in over ROS 2
   * 
   * Sends the data to the FusionEngineFramer
   * 
   * @param udp_msg The UDP message that came
   */
  void UdpCallback(const udp_msgs::msg::UdpPacket::SharedPtr udp_msg);
  /**
   * @brief Function called when Serial message comes in over ROS 2
   * 
   * Sends the data to the FusionEngineFramer
   * 
   * @param udp_msg The Serial message that came
   */
  void SerialCallback(
      const std_msgs::msg::UInt8MultiArray::SharedPtr serial_msg);

  /**
   * @brief Function called by FusionEngineFramer
   * 
   * This function publishes the ROS 2 messages based off the header of the
   * incoming message
   * 
   * @param header Header of the message
   * @param payload_in Actual message to be reinterpretted to P1 structs
   */
  void PublishMessage(
      const point_one::fusion_engine::messages::MessageHeader& header,
      const void* payload_in);

  /**
   * @brief Frame ID of ROS 2 messages
   * 
   * Normally this should be mapped to the device name in URDF. However in the
   * case of Odometry or Pose messages this should be the map frame
   * 
   * TODO: (Siddharth): Split to device frame ID and map
   * 
   */
  std::string frame_id_{""};

  /**
   * @brief Pointer to Fusion Engine Framer object
   * 
   * Used to deserialize data packets coming in and to call the PublishMessage
   * function
   */
  std::unique_ptr<point_one::fusion_engine::parsers::FusionEngineFramer>
      framer_;

  /**
   * @brief The time we received the message
   * 
   * Since we are running this single threaded right now it would ideally be
   * synced with the message we are publishing.
   */
  rclcpp::Time msg_received_time_;
};
} // namespace ros_driver
} // namespace fusion_engine
} // namespace point_one

#endif // FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_NODE_HPP_