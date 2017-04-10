#include "irb120_vision.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

CSARAVision::CSARAVision()
{
  ros::NodeHandle nh;
  img_sub_ = nh.subscribe("usb_cam/image_raw", 1, &CSARAVision::cam_test, this);
}

void CSARAVision::cam_test(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO("Got image frame. Height: %d, Width: %d", img->height, img->width);
}
