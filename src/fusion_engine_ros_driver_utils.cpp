//  Copyright 2023 Siddharth Saha
/**
 * @brief Source file containing implementation of functions in Fusion Engine ROS Driver utils header file
*/

#include "fusion_engine_ros_driver/fusion_engine_ros_driver_utils.hpp"

#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"

namespace point_one
{
namespace fusion_engine
{
namespace ros_utils
{

void toGPSFix(const point_one::fusion_engine::messages::ros::GPSFixMessage & contents, gps_msgs::msg::GPSFix::SharedPtr & gps_fix_msg) {
    gps_fix_msg->latitude  = contents.latitude_deg;
    gps_fix_msg->longitude = contents.longitude_deg;
    gps_fix_msg->altitude  = contents.altitude_m;
    gps_fix_msg->track     = contents.track_deg;
    gps_fix_msg->speed     = contents.speed_mps;
    gps_fix_msg->climb     = contents.climb_mps;
    gps_fix_msg->pitch     = contents.pitch_deg;
    gps_fix_msg->roll      = contents.roll_deg;
    gps_fix_msg->dip       = contents.dip_deg;
    gps_fix_msg->time      = contents.p1_time.seconds + (contents.p1_time.fraction_ns * 1e-9); // time since power-on
    gps_fix_msg->gdop      = contents.gdop;
    gps_fix_msg->hdop      = contents.hdop;
    gps_fix_msg->vdop      = contents.vdop;
    gps_fix_msg->tdop      = contents.tdop;
    gps_fix_msg->err       = contents.err_3d_m;
    gps_fix_msg->err_horz  = contents.err_horiz_m;
    gps_fix_msg->err_vert  = contents.err_vert_m;
    gps_fix_msg->err_speed = contents.err_speed_mps;
    gps_fix_msg->err_climb = contents.err_climb_mps;
    gps_fix_msg->err_time  = contents.err_time_sec;
    gps_fix_msg->err_pitch = contents.err_pitch_deg;
    gps_fix_msg->err_roll  = contents.err_roll_deg;
    gps_fix_msg->err_dip   = contents.err_dip_deg;
    std::copy(std::begin(contents.position_covariance_m2), std::end(contents.position_covariance_m2), std::begin(gps_fix_msg->position_covariance));
    gps_fix_msg->position_covariance_type = contents.position_covariance_type;
}

void toNavSatFix(const point_one::fusion_engine::messages::ros::GPSFixMessage & contents, sensor_msgs::msg::NavSatFix::SharedPtr & nav_sat_fix_msg) {
    // fix.header = gps_fix.header;
    // fix.status.status = gps_fix.status.status;
    // fix.status.service = 0;
    // if (gps_fix.status.position_source & gps_msgs::msg::GPSStatus::SOURCE_GPS)
    // {
    //   fix.status.service = fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    // }
    // if (gps_fix.status.orientation_source & gps_msgs::msg::GPSStatus::SOURCE_MAGNETIC)
    // {
    //   fix.status.service = fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    // }
    nav_sat_fix_msg->latitude  = contents.latitude_deg;
    nav_sat_fix_msg->longitude = contents.longitude_deg;
    nav_sat_fix_msg->altitude  = contents.altitude_m;
    std::copy(std::begin(contents.position_covariance_m2), std::end(contents.position_covariance_m2), std::begin(nav_sat_fix_msg->position_covariance));
    nav_sat_fix_msg->position_covariance_type = contents.position_covariance_type;
}

void toImu(const point_one::fusion_engine::messages::ros::IMUMessage & contents, sensor_msgs::msg::Imu::SharedPtr & imu_msg) {
    imu_msg->orientation.x = contents.orientation[0];
    imu_msg->orientation.y = contents.orientation[1];
    imu_msg->orientation.z = contents.orientation[2];
    imu_msg->orientation.w = contents.orientation[3];
    std::copy(std::begin(contents.orientation_covariance), std::end(contents.orientation_covariance), std::begin(imu_msg->orientation_covariance));
    imu_msg->angular_velocity.x = contents.angular_velocity_rps[0];
    imu_msg->angular_velocity.y = contents.angular_velocity_rps[1];
    imu_msg->angular_velocity.z = contents.angular_velocity_rps[2];
    std::copy(std::begin(contents.angular_velocity_rps), std::end(contents.angular_velocity_rps), std::begin(imu_msg->angular_velocity_covariance));
    imu_msg->linear_acceleration.x = contents.acceleration_mps2[0];
    imu_msg->linear_acceleration.y = contents.acceleration_mps2[1];
    imu_msg->linear_acceleration.z = contents.acceleration_mps2[2];
    std::copy(std::begin(contents.acceleration_covariance), std::end(contents.acceleration_covariance), std::begin(imu_msg->linear_acceleration_covariance));
}

void toP1ROSPose(const point_one::fusion_engine::messages::ros::PoseMessage & contents, geometry_msgs::msg::PoseStamped::SharedPtr & pose_msg) {
    pose_msg->pose.position.x = contents.position_rel_m[0];
    pose_msg->pose.position.y = contents.position_rel_m[1];
    pose_msg->pose.position.z = contents.position_rel_m[2];
    pose_msg->pose.orientation.x = contents.orientation[0];
    pose_msg->pose.orientation.y = contents.orientation[1];
    pose_msg->pose.orientation.z = contents.orientation[2];
    pose_msg->pose.orientation.w = contents.orientation[3];
}

void SendDataToFramer(const std::vector<uint8_t> & p1_data, std::unique_ptr<point_one::fusion_engine::parsers::FusionEngineFramer> & framer) {
    if (!p1_data.empty()) {
        const uint8_t * buffer = p1_data.data();
        framer->OnData(buffer, sizeof(uint8_t) * p1_data.size());
    }
}
}
}
}