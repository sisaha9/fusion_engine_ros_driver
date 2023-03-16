//  Copyright 2023 Siddharth Saha
/**
 * @brief Header file containing definition of utility functions
*/

#ifndef FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_UTILS_HPP_
#define FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_ROS_DRIVER_UTILS_HPP_

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "point_one/fusion_engine/parsers/fusion_engine_framer.h"
#include "point_one/fusion_engine/messages/ros.h"
#include "point_one/fusion_engine/messages/solution.h"

namespace point_one
{
namespace fusion_engine
{
namespace ros_utils
{

void toGPSFix(const point_one::fusion_engine::messages::ros::GPSFixMessage & contents, gps_msgs::msg::GPSFix::SharedPtr & gps_fix_msg);

void toNavSatFix(const point_one::fusion_engine::messages::ros::GPSFixMessage & contents, sensor_msgs::msg::NavSatFix::SharedPtr & nav_sat_fix_msg);

void toImu(const point_one::fusion_engine::messages::ros::IMUMessage & contents, sensor_msgs::msg::Imu::SharedPtr & imu_msg);

void toP1ROSPose(const point_one::fusion_engine::messages::ros::PoseMessage & contents, geometry_msgs::msg::PoseStamped::SharedPtr & pose_msg);

void SendDataToFramer(const std::vector<uint8_t> & p1_data, std::unique_ptr<point_one::fusion_engine::parsers::FusionEngineFramer> & framer);
}
}
}

#endif