#ifndef CALIBRATEDVIDEO
#define CALIBRATEDVIDEO

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
//#include <std_msgs/String.h>

class CalibratedVideo{
public:
	CalibratedVideo(ros::NodeHandle n);

  // Callback function.
	void send_video(const sensor_msgs::ImageConstPtr& img);
//	void send_video(const std_msgs::String::ConstPtr& msg);
	image_transport::Subscriber img_sub;
//	ros::Subscriber img_sub;
	image_transport::Publisher img_pub;
//	ros::Publisher img_pub;	
	cv::Mat cameraMatrix, distCoeffs;
//	cv::VideoCapture capture(0);

};


#endif
