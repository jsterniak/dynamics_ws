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
  uint32_t erode_attempts = 4U;

  // set up blob detector
  cv::SimpleBlobDetector::Params params;

  params.filterByArea = true;
  params.minArea = 900;
//    params.minThreshold = 0;
//    params.maxThreshold = 240;
  cv::SimpleBlobDetector detector(params);

  // iterate until only the fiducials are identified
  cv::Mat k_erod, img_erod, img_g, img_g_inv, img_g_blur, img_fiducials;
  while (fiducial_count != 3U && erode_attempts < 11U)
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

  if (0U < fiducial_count)
  {
    // add keypoints to image
    cv::drawKeypoints(img_g_inv, fiducials, img_fiducials, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // publish image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_fiducials).toImageMsg();
    pcb_feature_pub_.publish(msg);
  }
  else
  {
    ROS_ERROR("fiducials: %d, erosion_attempts: %d", fiducial_count, erode_attempts);
  }
}

void CSARAVision::find_triangle_base_pts(const point_vec& fiducials, point_vec& base_pts)
{
  // isolate head point
  const int8_t head_idx = min_y_index(fiducials);

  // clear base_pts and add the points that aren't head points
  base_pts.clear();
  for (uint8_t it_point = 0u; it_point<fiducials.size(); ++it_point)
  {
    if(head_idx != it_point)
    {
      cv::Point2d pt;
      pt.x = fiducials[it_point].x;
      pt.y = fiducials[it_point].y;
      base_pts.push_back(pt);
    }
    else
    {
      // do nothing
    }
  }
}

int8_t CSARAVision::min_y_index(const point_vec& fiducials)
{
  // need fiducials
  ROS_ASSERT(!fiducials.empty());

  double_t min_y = fiducials[0].y;
  int8_t min_idx = 0;

  for (uint8_t it_point = 1u; it_point <fiducials.size(); ++it_point)
  {
    if (fiducials[it_point].y < min_y)
    {
      min_idx = it_point;
      min_y = fiducials[it_point].y;
    }
    else
    {
      // do nothing
    }
  }

  return min_idx;
}

double_t CSARAVision::calc_rotation(const point_vec& base_pts)
{
  // only 2 base points allowed
  ROS_ASSERT(2U == base_pts.size());

  double_t dx = base_pts[1].x - base_pts[0].x;
  double_t dy = base_pts[1].y - base_pts[0].y;
  if (fabs(dx) <  std::numeric_limits<double_t>::epsilon())
  {
    return M_PI/2;
  }
  else
  {
    // y is inverted in pixel space so invert angle
    return -atan(dy/dx);
  }

}

double_t CSARAVision::calc_perim(const point_vec& pts)
{
  // only works for triangles
  ROS_ASSERT(3U == pts.size());

  // sum up all distances between points
  return calc_dist(pts[0],pts[1]) + calc_dist(pts[1],pts[2]) + calc_dist(pts[2],pts[0]);
}

double_t CSARAVision::calc_dist(const cv::Point2d& pt1, const cv::Point2d& pt2)
{
  double_t dx = pt1.x - pt2.x;
  double_t dy = pt1.y - pt2.y;

  double_t dist_squared = dx*dx + dy*dy;

  return sqrt(dist_squared);
}

void CSARAVision::calc_centroid(const point_vec& pts, cv::Point2d& centroid)
{
  // should have three points
  ROS_ASSERT(!pts.empty());

  uint32_t point_count = 0;
  double_t x_sum = 0;
  double_t y_sum = 0;
  for (point_vec::const_iterator it_pt = pts.begin(); it_pt != pts.end(); ++it_pt)
  {
    x_sum += it_pt->x;
    y_sum += it_pt->y;
    point_count++;
  }
  centroid.x = x_sum/point_count;
  centroid.y = y_sum/point_count;
}

void CSARAVision::calc_translation(const cv::Point2d& ref_centroid, const cv::Point2d& img_centroid,
                                   const double_t ref_perimeter, const double_t img_perimeter,
                                   const int32_t img_rows, const int32_t img_cols,
                                   double_t& trans_x, double_t& trans_y)
{
  // need positive perimeter
  ROS_ASSERT(img_perimeter >  std::numeric_limits<double_t>::epsilon());

  // convert from pixel scale to meters
  const double_t m_per_px = ref_perimeter / img_perimeter;

  // determine offset of shape centroid to image center
  cv::Point2d img_center;
  img_center.x = 0.5*static_cast<double_t>(img_cols);
  img_center.y = 0.5*static_cast<double_t>(img_rows);

  const double_t trans_x_px = img_centroid.x - img_center.x;
  const double_t trans_y_px = img_centroid.y - img_center.y;

  // convert to world space and determine offset of centroid vs. nominal offset
  trans_x = (trans_x_px * m_per_px) - ref_centroid.x;
  trans_y = (trans_y_px * m_per_px) - ref_centroid.y;
}

void CSARAVision::fiducial_to_pt_vec(const fiducial_vec& fiducials, point_vec& pts)
{
  for (fiducial_vec::const_iterator it_point = fiducials.begin(); it_point != fiducials.end(); ++it_point)
  {
    cv::Point2d pt;
    pt.x = it_point->pt.x;
    pt.y = it_point->pt.y;
    pts.push_back(pt);
  }
}

void CSARAVision::register_pcb(const fiducial_vec& img_fiducials, int32_t img_rows, int32_t img_cols)
{
  // registration requires 3 fiducial points
  ROS_ASSERT(3U == img_fiducials.size());

  // convert from keypoints to points
  point_vec image_pts;
  fiducial_to_pt_vec(img_fiducials, image_pts);

  // isolate the base of the triangle from fiducials
  point_vec base_pts;
  find_triangle_base_pts(image_pts, base_pts);

  // determine offset angle from triangle base angle
  const double_t rotation_angle = calc_rotation(base_pts);

  // hard coded fiducial locations
  point_vec model_pts;
  model_pts.push_back(cv::Point2d(-0.00607,0.00577));
  model_pts.push_back(cv::Point2d(0.00612, 0.00574));
  model_pts.push_back(cv::Point2d(0.0, -0.00645));

  // compute perimeters for scale calculation
  const double_t image_perim = calc_perim(image_pts);
  const double_t model_perim = calc_perim(model_pts);

  // find centroid of triangles
  cv::Point2d image_centroid;
  calc_centroid(image_pts, image_centroid);
  cv::Point2d model_centroid;
  calc_centroid(model_pts, model_centroid);

  // compute translation in world space
  double_t translation_x, translation_y;
  calc_translation(model_centroid, image_centroid,
                   model_perim, image_perim,
                   img_rows, img_cols,
                   translation_x, translation_y);

//  ROS_ERROR("img:");
//  ROS_ERROR("%f, %f",img_pts[0].x, img_pts[0].y);
//  ROS_ERROR("%f, %f",img_pts[1].x, img_pts[1].y);
//  ROS_ERROR("%f, %f",img_pts[2].x, img_pts[2].y);
//  ROS_ERROR("%f, %f",img_pts[3].x, img_pts[3].y);

//  ROS_ERROR("model:");
//  ROS_ERROR("%f, %f",model_pts[0].x, model_pts[0].y);
//  ROS_ERROR("%f, %f",model_pts[1].x, model_pts[1].y);
//  ROS_ERROR("%f, %f",model_pts[2].x, model_pts[2].y);
//  ROS_ERROR("%f, %f",model_pts[3].x, model_pts[3].y);

  // publish pcb pose
  geometry_msgs::Pose2D pose_msg;
  pose_msg.x = translation_x;
  pose_msg.y = translation_y;
  pose_msg.theta = rotation_angle;
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
