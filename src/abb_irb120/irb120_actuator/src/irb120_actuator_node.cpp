#include "irb120_actuator.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irb120_actuator");

  //  CIRBActuator t_actuator("/dev/ttyUSB0", 9600U);
  CIRBActuator t_actuator("/dev/pts/2", 9600U);

  ros::Rate t_loop_rate(1); // 1 Hz

  while (ros::ok())
  {
    // check for parameter update and send
    t_actuator.update_and_send(false);

    t_loop_rate.sleep();
    ros::spinOnce();
  }

}
