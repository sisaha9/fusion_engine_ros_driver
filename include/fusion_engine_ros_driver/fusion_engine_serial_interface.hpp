//  Copyright 2023 Siddharth Saha
/**
 * @brief Header file containing definition of serial interface class
*/

#ifndef FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_SERIAL_INTERFACE_NODE_HPP_
#define FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_SERIAL_INTERFACE_NODE_HPP_

#include <string>
#include <memory>

#include "point_one/fusion_engine/parsers/fusion_engine_framer.h"
#include "serial_driver/serial_driver.hpp"
#include "rclcpp/logger.hpp"

namespace point_one
{
namespace fusion_engine
{
namespace ros_serial_interface
{

#define NUM_THREADS 2
class FusionEngineSerialInterface
{
public:
    explicit FusionEngineSerialInterface(const std::shared_ptr<point_one::fusion_engine::parsers::FusionEngineFramer> framer, const rclcpp::Logger & logger, std::string & device_name, const std::string & flow_control, const std::string & parity, const std::string & stop_bits, const uint32_t baud_rate);
    ~FusionEngineSerialInterface();

private:
    bool InitPort(const std::string & device_name);
    void InitDeviceConfig(const std::string & flow_control, const std::string & parity, const std::string & stop_bits, const uint32_t baud_rate);
    void SerialCallback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);
    uint32_t baud_rate_;

    std::unique_ptr<drivers::common::IoContext> owned_ctx_{};
    std::string device_name_{};
    drivers::serial_driver::FlowControl flow_control_{drivers::serial_driver::FlowControl::NONE};
    drivers::serial_driver::Parity parity_{drivers::serial_driver::Parity::NONE};
    drivers::serial_driver::StopBits stop_bits_{drivers::serial_driver::StopBits::ONE};
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<point_one::fusion_engine::parsers::FusionEngineFramer> framer_;

    rclcpp::Logger logger_;
};
}
}
}

#endif
