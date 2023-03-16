//  Copyright 2023 Siddharth Saha
/**
 * @brief Header file containing definition of serial interface class
*/

#ifndef FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_SERIAL_INTERFACE_NODE_HPP_
#define FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_SERIAL_INTERFACE_NODE_HPP_

#include <string>
#include <memory>
#include "serial_driver/serial_driver.hpp"

namespace point_one
{
namespace fusion_engine
{
namespace ros_serial_interface
{
class FusionEngineSerialInterface
{
public:
    explicit FusionEngineSerialInterface(std::string & device_name, std::string & flow_control, std::string & parity, std::string & stop_bits);
    bool init_port();
    

private:
    std::unique_ptr<IoContext> owned_ctx_{};
    std::string device_name_{};
    std::string flow_control_{};
    std::string parity_{};
    std::string stop_bits_{};
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
};
}
}
}

#endif
