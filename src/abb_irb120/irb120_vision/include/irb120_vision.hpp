#ifndef IRB120_VISION_HPP_DEF
#define IRB120_VISION_HPP_DEF

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>

class CSARAVision
{
public:
  CSARAVision();
  void state_update_cb(const std_msgs::Int32ConstPtr& msg);
  void img_frame_cb(const sensor_msgs::ImageConstPtr& img);

private:
  int32_t robot_state_cur_;
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Subscriber state_sub_;
  image_transport::Publisher pcb_feature_pub_;
  image_transport::ImageTransport it_;
};

#endif
