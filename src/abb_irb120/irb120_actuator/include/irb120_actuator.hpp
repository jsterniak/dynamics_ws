#ifndef IRB120_ACTUATOR_HPP
#define IRB120_ACTUATOR_HPP

#include <ros/ros.h>
#include <serial/serial.h>

class CIRBActuator
{
public:
  CIRBActuator(std::string port_name, unsigned long baud);

  void update_and_send(const bool f_force_send);

  void send_byte(const bool f_force_send, int t_send_byte);

private:
  void send();

  ros::NodeHandle nh_;

  serial::Serial serial_;

  uint8_t send_byte_;
};

#endif
