#include "irb120_actuator.hpp"
#include <ros/ros.h>
#include <string>
#include <serial/serial.h>

CIRBActuator::CIRBActuator(std::string port_name, unsigned long baud)
  : serial_(port_name, baud)
  , send_byte_(0x00)
  , nh_()
{
  ROS_ASSERT(serial_.isOpen());

  // ensure first message gets out
  update_and_send(true);
}

void CIRBActuator::update_and_send(const bool f_force_send)
{
  const unsigned char t_old_byte(send_byte_);

  int t_send_byte;
  nh_.param("/irb120_actuator/send_byte", t_send_byte, 0x00);
  ROS_ASSERT(t_send_byte >= 0U && t_send_byte <= 255U);
  send_byte_ = static_cast<unsigned char>(t_send_byte);

  if (t_old_byte != send_byte_ || f_force_send)
  {
    std::string t_string(1, send_byte_);

    size_t t_bytes_written = serial_.write(t_string);

    ROS_INFO("sent %d bytes", t_bytes_written);
  }
  else
  {
    // do nothing
  }
}
