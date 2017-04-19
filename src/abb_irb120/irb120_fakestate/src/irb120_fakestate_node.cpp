#include "irb120_fakestate.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irb120_fakestate");

  CIRBFakeState t_fakestate;

  ros::Rate t_loop_rate(20); // 50 ms

  while (ros::ok())
  {
    // check for parameter update and send
    t_fakestate.update_and_send();

    t_loop_rate.sleep();
    ros::spinOnce();
  }

}
