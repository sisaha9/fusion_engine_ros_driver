//  Copyright 2023 Siddharth Saha
/**
 * @brief Source file containing implementation of functions in Fusion Engine
 * Serial Interface header file
*/

#include "fusion_engine_ros_driver/fusion_engine_serial_interface.hpp"

#include <string>

#include "rclcpp/logging.hpp"

namespace point_one {
namespace fusion_engine {
namespace ros_serial_interface {
FusionEngineSerialInterface::FusionEngineSerialInterface(const std::shared_ptr<point_one::fusion_engine::parsers::FusionEngineFramer> framer, const rclcpp::Logger & logger, std::string & device_name, const std::string & flow_control, const std::string & parity, const std::string & stop_bits, const uint32_t baud_rate) : logger_(logger)
{
    framer_ = framer;
    InitDeviceConfig(flow_control, parity, stop_bits, baud_rate);
    owned_ctx_ = std::make_unique<drivers::common::IoContext>(NUM_THREADS);
    serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
    InitPort(device_name);
}

FusionEngineSerialInterface::~FusionEngineSerialInterface()
{
    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void FusionEngineSerialInterface::InitDeviceConfig(const std::string & flow_control, const std::string & parity, const std::string & stop_bits, const uint32_t baud_rate)
{
    baud_rate_ = baud_rate;

    if (flow_control == "none") {
      flow_control_ = drivers::serial_driver::FlowControl::NONE;
    } else if (flow_control == "hardware") {
      flow_control_ = drivers::serial_driver::FlowControl::HARDWARE;
    } else if (flow_control == "software") {
      flow_control_ = drivers::serial_driver::FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
              "The flow_control parameter must be one of: none, software, or hardware."};
    }
    
    if (parity == "none") {
      parity_ = drivers::serial_driver::Parity::NONE;
    } else if (parity == "odd") {
      parity_ = drivers::serial_driver::Parity::ODD;
    } else if (parity == "even") {
      parity_ = drivers::serial_driver::Parity::EVEN;
    } else {
      throw std::invalid_argument{
              "The parity parameter must be one of: none, odd, or even."};
    }

    if (stop_bits == "1" || stop_bits == "1.0") {
      stop_bits_ = drivers::serial_driver::StopBits::ONE;
    } else if (stop_bits == "1.5") {
      stop_bits_ = drivers::serial_driver::StopBits::ONE_POINT_FIVE;
    } else if (stop_bits == "2" || stop_bits == "2.0") {
      stop_bits_ = drivers::serial_driver::StopBits::TWO;
    } else {
      throw std::invalid_argument{
              "The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }

    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate_, flow_control_, parity_, stop_bits_);
}

bool FusionEngineSerialInterface::InitPort(const std::string & device_name)
{
  device_name_ = device_name;
  try {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open()) {
        serial_driver_->port()->open();
        serial_driver_->port()->async_receive(
          std::bind(&FusionEngineSerialInterface::SerialCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
      }
  } catch (const std::exception & e) {
      RCLCPP_ERROR(
          logger_, "Error creating serial port: %s - %s",
      device_name_.c_str(), e.what());
  }
  return true;
}

void FusionEngineSerialInterface::SerialCallback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred)
{
  framer_->OnData(buffer.data(), bytes_transferred);
}
}
}
}