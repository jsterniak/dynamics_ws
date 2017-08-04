#ifndef BGAPLACEMENT_H
#define BGAPLACEMENT_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>

class BGAPlacement
{
public:
  void detectBGACallBack(const sensor_msgs::ImageConstPtr& img);
  BGAPlacement(ros::NodeHandle& n);
private:
  geometry_msgs::Point realWorldCoords;
  ros::Publisher m_orientationPub;
  ros::Subscriber m_imageSub;
  ros::Publisher m_xy_pickup_Pub;
  ros::Publisher m_xy_place_Pub;
  ros::Subscriber m_snapSub;
  image_transport::Publisher ic_outline_pub_;
  image_transport::ImageTransport it_;
};

#endif
