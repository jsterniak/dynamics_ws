#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include "bgaplacement.h"
#include <math.h> 
#include <string.h>
#include <std_msgs/Float64.h>
#include <iostream>

static const double hor_fov = 0.95; // in radians(= 54.43 degrees)
static const double heightCam = 30.0; //in mm

using namespace std;

BGAPlacement::BGAPlacement(ros::NodeHandle& n)
  : it_(n)
{
	m_orientationPub =  n.advertise<std_msgs:: Float64> ("/detect/bga/orientation",100);
	m_xy_pickup_Pub = n.advertise<geometry_msgs:: Point> ("/detect/bga_pickup/xy",100);
	m_xy_place_Pub = n.advertise<geometry_msgs:: Point> ("/detect/bga_place/xy",100);
	m_imageSub = n.subscribe("/CalibratedVideo/image_fine", 10, &BGAPlacement::detectBGACallBack,this);
  ic_outline_pub_ = it_.advertise("/CalibratedVideo/ic_outline_img", 1);
}

void BGAPlacement::detectBGACallBack(const sensor_msgs::ImageConstPtr& img)
{
	static int flag = 0;
	cv::Mat bgaOriginal, Bga_Chip_gray,Bga_Chip_thresh,imageFilter,imgBF,imageRGB, bwImg;
	cv::vector<cv::Point2f> coordsInCm;
	cv_bridge::CvImagePtr inMsgPtr;
	inMsgPtr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);

	bgaOriginal = inMsgPtr->image;
	int px = bgaOriginal.cols, py = bgaOriginal.rows;
	int horInPix=px;
	float pixPerUnit = px*.5/(heightCam*tan(hor_fov*.5));// calculated = 120.627
	cv::cvtColor(bgaOriginal, Bga_Chip_gray, CV_BGR2GRAY);
//	cv::threshold(Bga_Chip_gray,Bga_Chip_thresh, 140.0, 255.0, cv::THRESH_BINARY);	// PCB detection
	cv::threshold(Bga_Chip_gray,Bga_Chip_thresh, 45.0, 255.0, cv::THRESH_BINARY_INV);	// Chip detection


// ----------------------------------------------------------------------------------------------------------
	// Draw the min block.
/*	cv::vector<cv::vector<cv::Point> >  bwcontours;
	cv::threshold(Bga_Chip_gray,bwImg, 160.0, 255.0, cv::THRESH_BINARY);
	cv::findContours(bwImg,bwcontours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);  
	double maxArea = 0;
	vector<cv::Point> maxContour;  
	for(size_t i = 0; i < bwcontours.size(); i++)  
	{  
    	double area = cv::contourArea(bwcontours[i]);  
	    if (area > maxArea){  
        	maxArea = area;  
        	maxContour = bwcontours[i];
	    }  
	}
	cv::Mat drawing2 = cv::Mat::zeros( bgaOriginal.size(), CV_8UC3 );
	cv::Rect maxRect = cv::boundingRect(maxContour);  
	cv::Mat result2;  
	bwImg.copyTo(result2);  
	cv::rectangle(result2, maxRect, cv::Scalar(255));*/
// ----------------------------------------------------------------------------------------------------------




	cv::vector<cv::vector<cv::Point> > contours;
	cv:: bilateralFilter(Bga_Chip_thresh,imageFilter,9,75,75);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  
    cv::Mat imageFilter2;
    cv::erode(imageFilter,imageFilter2, element);
	cv::vector<cv::Vec4i> hierarchy	;
	cv::Mat imageCopy = imageFilter2.clone();

	cv::findContours(imageCopy,contours,hierarchy, cv::RETR_CCOMP ,cv::CHAIN_APPROX_NONE,cv::Point(0,0));
	cv::vector<cv::vector <cv::Point> > contours_poly(contours.size());
	cv::vector<cv::Rect> boundRect(contours.size());
	cv::Mat drawing = cv::Mat::zeros( bgaOriginal.size(), CV_8UC3 );
	cv::vector<int> width, height;
	cv::vector<cv::Point2f> points;
	int xc, yc;
	cv::vector<cv::RotatedRect> minRect( contours.size() );
	int itr = 0;
	geometry_msgs::Point realWorldCoords;
	geometry_msgs::Point rwc;
	std_msgs::Float64 orientation;
	rwc.x = 0;
	rwc.y = 0;
	for(unsigned int i = 0; i< contours.size(); i++)
	{
		double area =cv::contourArea(contours[i], true);
		cv::Moments moment = cv::moments((cv::Mat)contours[i]);

		cout << i << ": " << "Area: " << abs(area) << endl;	

		if (std::abs(area)>59000 & std::abs(area)<70000)	// The robot's tip position: 148.9mm, PCB detection
//		if (std::abs(area)>37000 & std::abs(area)<42000)	// The robot's tip position: 130.0mm, chip detection
		{

			// Check the area.
//			cout << i << ": " << "Area: " << abs(area) << endl;	

			boundRect[i] = cv::boundingRect(contours[i]);
			float w = boundRect[i].width, h = boundRect[i].height;

			// Check the ratio.
			cout << i << ": " << "Ratio: " << w/h << " " << h/w  << endl;

			if ( w/h>0.4 && w/h < 2.2)
			{
				// Calculate the position 
				points.push_back(cv::Point2f(moment.m10 / moment.m00, moment.m01 / moment.m00) );
				cv::circle(bgaOriginal, points.back(), 3, cv::Scalar(0,0,255), -1, 8, 0);
//				cv::rectangle( bgaOriginal, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0,0,255), 2, 8, 0 );
				cv::drawContours( bgaOriginal, contours, i, cv::Scalar(255,0,0), 1, 8, hierarchy, 0, cv::Point() );
				minRect[i] = cv::minAreaRect(contours[i]);
				float angle = minRect[i].angle;

//				cout << "enter here" << endl;

//				std::cout<<angle<<" deg"<<std::endl;
				orientation.data = angle;
				
				flag = flag + 1;
				cv::Point2f rect_points[4]; minRect[i].points( rect_points );
				for( int j = 0; j < 4; j++ )
				// Draw the rotated rectangle.
//					cv::line( drawing, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,255,255), 4, 8 );
					cv::line( bgaOriginal, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 4, 8 );


				rwc.x = ((points[itr].x-px*.5)/pixPerUnit);
				rwc.y = ((py*.5-points[itr].y)/pixPerUnit);
				rwc.z = heightCam;
//				cout<<"Contour centre pts "<<points[itr]<<endl;
//				cout<<"Real world coord "<<rwc<<endl;
//				cout<<"pix per unit "<<pixPerUnit<<endl;
//				cout<<"height is "<<heightCam*pixPerUnit<<endl;
			}
			// if(flag == 10)
			// {
			//    cout<<"Pickup Orientation "<<orientation.data<<endl;
			//    m_orientationPub.publish(orientation);

			// }	
			if(flag == 20)
			{
			   cout<<"Real world coords "<<rwc<<endl;
			   m_xy_pickup_Pub.publish(rwc);

			}

			if (flag == 70)
			{
				cout<<"Real world coords "<<rwc<<endl;
	            m_xy_place_Pub.publish(rwc);
				
			}			
			if(flag == 80)
			{
		   	    cout<<"Place Orientation "<<orientation.data<<endl;
				m_orientationPub.publish(orientation);

			}
			
		}
	}
	//cout<<flag<<endl;
//	cv::imshow("contours", bgaOriginal);
//	cv::imshow("contours", imageFilter2);
//	cv::waitKey(1);
  // publish image
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageFilter2).toImageMsg();
  ic_outline_pub_.publish(msg);
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "irb120_perception_node");
	ros::NodeHandle n;
	BGAPlacement detect(n);
	ros::spin();
}
