#include "irb120_vision.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>

CSARAVision::CSARAVision()
  : nh_()
  , it_(nh_)
{
  img_sub_ = nh_.subscribe("/CalibratedVideo/image_fine", 1, &CSARAVision::img_frame_cb, this);
  pcb_feature_pub_ = it_.advertise("/CalibratedVideo/pcb_feature_img", 1);
  pcb_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/irb120/pcb_pose",1000);
}

void CSARAVision::find_fiducials(const cv::Mat& im, fiducial_vec& fiducials)
{
  // erode image and find blobs for fiducials
  uint32_t fiducial_count = UINT_MAX;

  // initialize erosion counter for erosion attempts
  uint32_t erode_attempts = 0U;

  // set up blob detector
  cv::SimpleBlobDetector::Params params;

  params.filterByArea = true;
  params.minArea = 500;
//    params.minThreshold = 0;
//    params.maxThreshold = 240;
  cv::SimpleBlobDetector detector(params);

  // iterate until only the fiducials are identified
  cv::Mat k_erod, img_erod, img_g, img_g_inv, img_g_blur, img_fiducials;
  while (fiducial_count != 3U && erode_attempts < 10U)
  {
    // erode image to get rid of silkscreen and false features
    erode_attempts++;
    int32_t k_point = erode_attempts;
    int32_t k_size = 2 * erode_attempts + 1;

    k_erod = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_size,k_size), cv::Point(k_point,k_point));
    cv::erode(im, img_erod, k_erod);

    // convert to grayscale
    cv::cvtColor(img_erod, img_g, cv::COLOR_BGR2GRAY);

    // blur image
    cv::GaussianBlur(img_g, img_g_blur, cv::Size(15,15), 0, 0);

    // invert image
    img_g_inv = 255U - img_g_blur;

    // find blobs
    detector.detect(img_g_inv, fiducials);

    fiducial_count = fiducials.size();
  }

  if (3U == fiducial_count)
  {
    // add keypoints to image
    cv::drawKeypoints(im, fiducials, img_fiducials, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // publish image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_fiducials).toImageMsg();
    pcb_feature_pub_.publish(msg);
  }
  else
  {
    // do nothing
  }
}

void CSARAVision::register_pcb(const fiducial_vec& fiducials, int32_t img_rows, int32_t img_cols)
{
  if (3U != fiducials.size())
  {
    ROS_ERROR("PCB registration only configured for 3 fiducial points");
    return;
  }

  // convert keypoints to image point array
  // add centroid as fourth point since pcb is assumed flat and pnp requires 4 points
  std::vector<cv::Point2d> img_pts;
  uint32_t point_count = 0;
  double_t x_sum = 0;
  double_t y_sum = 0;
  for (fiducial_vec::const_iterator it_point = fiducials.begin(); it_point != fiducials.end(); ++it_point)
  {
    cv::Point2d pt;
    pt.x = it_point->pt.x;
    pt.y = it_point->pt.y;
    img_pts.push_back(pt);

    // compute statistics for centroid
    x_sum += pt.x;
    y_sum += pt.y;
    point_count++;
  }
  // append centroid, guaranteed to have 3 points (i.e. nonzero denom) from check at function entry
  cv::Point2d pt;
  pt.x = x_sum/point_count;
  pt.y = y_sum/point_count;
  img_pts.push_back(pt);

  // create actual PCB points in 3-D space from PCB cad file dimensions
  std::vector<cv::Point3d> model_pts;
  model_pts.push_back(cv::Point3d(-0.00607,0.00577,0.0));
  model_pts.push_back(cv::Point3d(0.00612, 0.00574, 0.0));
  model_pts.push_back(cv::Point3d(0.0, -0.00645, 0.0));
  // add centroid
  model_pts.push_back(cv::Point3d(0.0, 0.001687, 0.0));

  // camera model
  // image is undistorted already so assume ideal parameters
  double_t focal_length = static_cast<double_t>(img_cols);
  cv::Point2d center = cv::Point2d(static_cast<double_t>(img_cols)/2.0,static_cast<double_t>(img_rows)/2.0);
  cv::Mat camera_matrix = cv::Mat_<double_t>(3,3);
  camera_matrix.at<double_t>(0,0) = focal_length;
  camera_matrix.at<double_t>(0,1) = 0.0;
  camera_matrix.at<double_t>(0,2) = center.x;
  camera_matrix.at<double_t>(1,0) = 0.0;
  camera_matrix.at<double_t>(1,1) = focal_length;
  camera_matrix.at<double_t>(1,2) = center.y;
  camera_matrix.at<double_t>(2,0) = 0.0;
  camera_matrix.at<double_t>(2,1) = 0.0;
  camera_matrix.at<double_t>(2,2) = 1.0;

  cv::Mat distortion_coeff = cv::Mat::zeros(4,1,cv::DataType<double_t>::type);

  // convert from pixel to world space
  cv::Mat rotation_vec;
  cv::Mat translation_vec;

  bool success = cv::solvePnP(model_pts, img_pts, camera_matrix, distortion_coeff, rotation_vec, translation_vec);

  cv::Mat rotation_mat;
  cv::Rodrigues(rotation_vec, rotation_mat);

  // assume any rotation is only about z
  double_t z_rot_angle = acos(0.5*fabs(rotation_mat.at<double_t>(0,0))+0.5*fabs(rotation_mat.at<double_t>(1,1)));
  // sign correct rotation and translation
  if (sgn(rotation_mat.at<double_t>(0,1)) == sgn(rotation_mat.at<double_t>(1,0))
      || rotation_mat.at<double_t>(1,0) > 0.0)
  {
    z_rot_angle = -z_rot_angle;
    translation_vec = -translation_vec;
  }
  else
  {
    // do nothing
  }
  double_t x_ofs = translation_vec.at<double_t>(0,0);
  double_t y_ofs = translation_vec.at<double_t>(1,0);

  // publish pcb pose
  geometry_msgs::Pose2D pose_msg;
  pose_msg.x = x_ofs;
  pose_msg.y = y_ofs;
  pose_msg.theta = z_rot_angle;
  pcb_pose_pub_.publish(pose_msg);
}

void CSARAVision::img_frame_cb(const sensor_msgs::ImageConstPtr& img)
{
  // read image into cv Mat
  cv::Mat im;
  im = cv_bridge::toCvCopy(img, "bgr8")->image;

  // locate fiducial markings
  fiducial_vec fiducials;
  find_fiducials(im, fiducials);

  // register pcb if correct number of fiducials were found
  if (3U == fiducials.size())
  {
    // determine pcb location in world coordinates
    register_pcb(fiducials, im.rows, im.cols);
  }
  else
  {
    // do nothing
  }
}
