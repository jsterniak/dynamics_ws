#ifndef IRB120_ACTUATOR_HPP
#define IRB120_ACTUATOR_HPP

#include <ros/ros.h>
#include <serial/serial.h>

class CIRBActuator
{
public:
  CIRBActuator(std::string port_name, unsigned long baud);

  void update_and_send(const bool f_force_send);

private:
  void send();

  ros::NodeHandle nh_;

  serial::Serial serial_;

  uint8_t send_byte_;
};

#endif
