#ifndef IRB120_VISION_HPP_DEF
#define IRB120_VISION_HPP_DEF

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class CSARAVision
{
public:
  CSARAVision();

  typedef std::vector<cv::KeyPoint> fiducial_vec;

  void state_update_cb(const std_msgs::Int32ConstPtr& msg);
  void img_frame_cb(const sensor_msgs::ImageConstPtr& img);

private:
  void find_fiducials(const cv::Mat& im, fiducial_vec& fiducials);
  void register_pcb(const fiducial_vec& fiducials, int32_t img_rows, int32_t img_cols);

  template<typename T>
  int32_t sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  image_transport::Publisher pcb_feature_pub_;
  image_transport::ImageTransport it_;
  ros::Publisher pcb_pose_pub_;
};

#endif
