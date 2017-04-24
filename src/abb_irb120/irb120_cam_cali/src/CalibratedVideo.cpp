#include "CalibratedVideo.h"

#include <cv_bridge/cv_bridge.h>
//#include <sstream>
using namespace cv;
using namespace std;


CalibratedVideo::CalibratedVideo(ros::NodeHandle n){
	image_transport::ImageTransport it(n);
	img_sub = it.subscribe("/usb_cam/image_raw", 1, &CalibratedVideo::send_video, this);
//	img_sub = n.subscribe("chatter", 1, CalibratedVideo::send_video, this);
	img_pub = it.advertise("/CalibratedVideo/image_fine",1);
//	img_pub = n.advertise<sensor_msgs::Image>("/CalibratedVideo/image_fine", 100);
	

	//read the calibrating parameters.
	string path = ros::package::getPath("irb120_cam_cali");
	const string inputSettingsFile = path + "/src/out_camera_data.xml";
	ROS_INFO("%s\n", inputSettingsFile.c_str());
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        ROS_INFO("Could not open the configuration file");
        return;
    }
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	fs.release();
//	for (int i=0;i<5;i++){
//		ROS_INFO("%f ", distCoeffs.at<double>(i));
//	}
//	ROS_INFO("\n");

}


void CalibratedVideo::send_video(const sensor_msgs::ImageConstPtr& img){
//void CalibratedVideo::send_video(const std_msgs::String::ConstPtr& msg){
	Mat rawFrame, newFrame;
	cv_bridge::CvImagePtr inMsgPtr;
	inMsgPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

	rawFrame = inMsgPtr->image;
//	capture>>rawFrame;

//	capture.read(rawFrame);
	cv::undistort(rawFrame, newFrame, cameraMatrix, distCoeffs);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", newFrame).toImageMsg();
	img_pub.publish(msg);

//	for (int i=300; i<305;i++){
//			ROS_INFO("%d %d %d %d %d", (int)rawFrame.at<uchar>(300,i),(int)rawFrame.at<uchar>(301,i),(int)rawFrame.at<uchar>(302,i),(int)rawFrame.at<uchar>(303,i),(int)rawFrame.at<uchar>(304,i));
//	}
//ROS_INFO("%d %d", rawFrame.rows, rawFrame.cols);
}

int main (int argc, char* argv[]){
	ros::init(argc, argv, "cali");
	ros::NodeHandle nh;
	CalibratedVideo cali(nh);
	ros::spin();

	return 0;
}



