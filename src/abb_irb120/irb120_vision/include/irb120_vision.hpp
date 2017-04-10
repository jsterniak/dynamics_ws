#ifndef IRB120_VISION_HPP_DEF
#define IRB120_VISION_HPP_DEF

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

class CSARAVision
{
public:
  CSARAVision();
  void cam_test(const sensor_msgs::ImageConstPtr& img);

private:
  ros::Subscriber img_sub_;
};

#endif
