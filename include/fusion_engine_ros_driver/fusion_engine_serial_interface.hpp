//  Copyright 2023 Siddharth Saha
/**
 * @brief Header file containing definition of serial interface class
*/

#ifndef FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_SERIAL_INTERFACE_NODE_HPP_
#define FUSION_ENGINE_ROS_DRIVER__FUSION_ENGINE_SERIAL_INTERFACE_NODE_HPP_

#include <string>
#include <memory>
#include "udp_driver/udp_driver.hpp"

namespace point_one
{
namespace fusion_engine
{
namespace ros_serial_interface
{
class FusionEngineSerialInterface
{
public:
    explicit FusionEngineSerialInterface(std::string & usb_port);
private:
    std::unique_ptr<
};
}
}
}

#endif
