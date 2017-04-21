#include "CalibratedVideo.h"

#include <cv_bridge/cv_bridge.h>
//#include <opencv2/core/core.hpp>


using namespace cv;
using namespace std;


CalibratedVideo::CalibratedVideo(ros::NodeHandle n){
	img_sub = n.subscribe("/usb_cam/image_raw", 10, &CalibratedVideo::send_video, this);
	img_pub = n.advertise<sensor_msgs::Image>("/CalibratedVideo/image_fine",100);
	
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
}

void CalibratedVideo::send_video(const sensor_msgs::ImageConstPtr& img){
	Mat newFrame;
	cv_bridge::CvImagePtr inMsgPtr;
	inMsgPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

	undistort(inMsgPtr->image, newFrame, cameraMatrix, distCoeffs);
	inMsgPtr->image = newFrame;
	img_pub.publish(inMsgPtr->toImageMsg());

	ROS_INFO("sent frame width=%d height=%d", inMsgPtr->image.cols, inMsgPtr->image.rows);
}

int main (int argc, char* argv[]){
	ros::init(argc, argv, "cali");
	ros::NodeHandle nh;
	CalibratedVideo cali(nh);
	ros::spin();

	return 0;
}
