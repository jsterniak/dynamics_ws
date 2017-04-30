#include "irb120_vision.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

CSARAVision::CSARAVision()
  : robot_state_cur_(0)
  , nh_()
  , it_(nh_)
{
  img_sub_ = nh_.subscribe("/CalibratedVideo/image_fine", 1, &CSARAVision::img_frame_cb, this);
  state_sub_ = nh_.subscribe("/irb120/robot_state", 1, &CSARAVision::state_update_cb, this);
  pcb_feature_pub_ = it_.advertise("/CalibratedVideo/pcb_feature_img", 1);
}

void CSARAVision::state_update_cb(const std_msgs::Int32ConstPtr& msg)
{
  robot_state_cur_ = msg->data;
}

void CSARAVision::img_frame_cb(const sensor_msgs::ImageConstPtr& img)
{
//  if (IRBStateMachine::DetectPCB == robot_state_cur_)
  {
    // read image into cv Mat
    cv::Mat im;
    im = cv_bridge::toCvCopy(img, "bgr8")->image;

    // erode image and find blobs for fiducials
    uint32_t blob_count = UINT_MAX;

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
    cv::Mat k_erod, img_erod, img_g, img_g_inv, img_g_blur, img_blob;
    std::vector<cv::KeyPoint> blobs;
    while (blob_count != 3U && erode_attempts < 7)
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
      detector.detect(img_g_inv, blobs);

      blob_count = blobs.size();
    }

    if (3U == blob_count)
    {
      // add keypoints to image
      cv::drawKeypoints(im, blobs, img_blob, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      // publish image
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_blob).toImageMsg();
      pcb_feature_pub_.publish(msg);
    }
    else
    {
      // do nothing
    }
  }
}
