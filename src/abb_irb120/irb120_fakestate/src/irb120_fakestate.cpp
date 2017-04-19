#include "irb120_fakestate.hpp"
#include "irb120_mover/robot_state.h"
#include "state_machine.hpp"
#include <ros/ros.h>

CIRBFakeState::CIRBFakeState()
  : nh_()
{
  state_pub_ = nh_.advertise<irb120_mover::robot_state>("/irb120/robot_state", 1000);
}

void CIRBFakeState::update_and_send()
{
  int32_t t_new_state;

  nh_.param("/fake_state", t_new_state, static_cast<int32_t>(IRBStateMachine::Initialization));

  // send new state
  irb120_mover::robot_state state_msg;
  state_msg.actual_state = t_new_state;
  state_pub_.publish(state_msg);
}
