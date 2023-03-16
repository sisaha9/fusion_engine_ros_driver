//  Copyright 2023 Siddharth Saha
/**
 * @brief Header file containing definition of utility functions
*/

#ifndef FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_UTILS_HPP_
#define FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_UTILS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "point_one/fusion_engine/messages/ros.h"
#include "point_one/fusion_engine/messages/solution.h"
#include "point_one/fusion_engine/parsers/fusion_engine_framer.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace point_one
{
namespace fusion_engine
{
namespace ros_utils
{
/**
 * @brief Fills ROS 2 GPSFix message with values from P1 GPSFixMessage struct
 * 
 * @param contents P1 GPSixMessage struct matching contents of ROS 2 GPSFix
 * message
 * @param gps_fix_msg Pointer to ROS 2 GPS Fix Message. Needs to be filled
 */
void toGPSFix(
  const point_one::fusion_engine::messages::ros::GPSFixMessage & contents,
  gps_msgs::msg::GPSFix::SharedPtr & gps_fix_msg);

/**
 * @brief Fills ROS 2 NavSatFix message with values from P1 GPSFixMessage struct
 * 
 * @param contents P1 GPSixMessage struct matching contents of ROS 2 GPSFix
 * message
 * @param nav_sat_fix_msg Pointer to ROS 2 Nav Sat Fix Message. Needs to be
 * filled
 */
void toNavSatFix(
  const point_one::fusion_engine::messages::ros::GPSFixMessage & contents,
  sensor_msgs::msg::NavSatFix::SharedPtr & nav_sat_fix_msg);

/**
 * @brief Fills ROS 2 IMU message with values from P1 IMUMessage struct
 * 
 * @param contents P1 IMUMessage struct matching contents of ROS 2 IMU message
 * @param imu_msg Pointer to ROS 2 IMU message. Needs to be filled
 */
void toImu(
  const point_one::fusion_engine::messages::ros::IMUMessage & contents,
  sensor_msgs::msg::Imu::SharedPtr & imu_msg);

/**
 * @brief Fills ROS 2 Pose message with values from P1 ROS Pose struct
 * 
 * @param contents P1 PoseMessage struct matching contents of ROS 2 Pose message
 * @param pose_msg Pointer to ROS 2 Pose message. Needs to be filled
 */
void toP1ROSPose(
  const point_one::fusion_engine::messages::ros::PoseMessage & contents,
  geometry_msgs::msg::PoseStamped::SharedPtr & pose_msg);

/**
 * @brief Feeds a vector of data from FusionEngine server to the
 * FusionEngineFramer
 * 
 * @param fusion_engine_server_data A vector of data from a FusionEngine server
 * @param framer FusionEngineFramer that takes data and calls a callback
 * function on receiving data
 * 
 * @note In this case the function framer calls is PublishMessage in
 * fusion_engine_ros_driver_node.cpp and fusion_engine_ros_driver_node.hpp
 */
void SendDataToFramer(
  const std::vector<uint8_t> & fusion_engine_server_data,
  std::unique_ptr<point_one::fusion_engine::parsers::FusionEngineFramer> & framer);
}  // namespace ros_utils
}  // namespace fusion_engine
}  // namespace point_one

#endif