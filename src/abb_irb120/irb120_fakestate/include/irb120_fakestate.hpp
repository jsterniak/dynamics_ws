#ifndef IRB120_FAKESTATE_HPP
#define IRB120_FAKESTATE_HPP

#include <ros/ros.h>

class CIRBFakeState
{
public:
  CIRBFakeState();

  void update_and_send();

private:
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
};

#endif
