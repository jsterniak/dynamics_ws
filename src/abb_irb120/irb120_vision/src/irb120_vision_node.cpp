#include "irb120_vision.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irb120_vision");

  CSARAVision vision_node;

  ros::spin();

  return 0;
}
