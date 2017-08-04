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
  typedef std::vector<cv::Point2d> point_vec;

  void state_update_cb(const std_msgs::Int32ConstPtr& msg);
  void img_frame_cb(const sensor_msgs::ImageConstPtr& img);

private:
  void find_fiducials(const cv::Mat& im, fiducial_vec& fiducials);
  void register_pcb(const fiducial_vec& img_fiducials, int32_t img_rows, int32_t img_cols);
  void find_triangle_base_pts(const point_vec& fiducials, point_vec& base_pts);
  double_t calc_rotation(const point_vec& base_pts);
  double_t calc_perim(const point_vec& pts);
  void calc_centroid(const point_vec& pts, cv::Point2d& centroid);
  void calc_translation(const cv::Point2d& ref_centroid, const cv::Point2d& img_centroid,
                        const double_t ref_perimeter, const double_t img_perimeter,
                        const int32_t img_rows, const int32_t img_cols,
                        double_t& trans_x, double_t& trans_y);
  void fiducial_to_pt_vec(const fiducial_vec& fiducials, point_vec& pts);
  int8_t min_y_index(const point_vec& fiducials);
  double_t calc_dist(const cv::Point2d& pt1, const cv::Point2d& pt2);

  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  image_transport::Publisher pcb_feature_pub_;
  image_transport::ImageTransport it_;
  ros::Publisher pcb_pose_pub_;
};

#endif
