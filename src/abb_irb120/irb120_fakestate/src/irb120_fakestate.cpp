#include "irb120_fakestate.hpp"
#include "state_machine.hpp"
#include <std_msgs/Int32.h>
#include <ros/ros.h>

CIRBFakeState::CIRBFakeState()
  : nh_()
{
  state_pub_ = nh_.advertise<std_msgs::Int32>("/irb120/robot_state", 1000);
}

void CIRBFakeState::update_and_send()
{
  int32_t t_new_state;

  nh_.param("/fake_state", t_new_state, static_cast<int32_t>(IRBStateMachine::Initialization));

  // send new state
  std_msgs::Int32 state_msg;
  state_msg.data = t_new_state;
  state_pub_.publish(state_msg);
}
