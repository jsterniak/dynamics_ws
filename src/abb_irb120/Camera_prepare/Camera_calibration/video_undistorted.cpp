#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif


using namespace cv;
using namespace std;


int main(int argc, char* argv[])
{

    // 1.Read settings
    Mat cameraMatrix, distCoeffs;

    const string inputSettingsFile = argc > 1 ? argv[1] : "out_camera_data.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	fs.release();

	VideoCapture capture(0);
	if(!capture.isOpened())
		return -1;
		

	capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);


	// -------------------- Capture frame from video ----------------------------
	Mat rawFrame, frame_new;

	namedWindow("Camera Window", 1);
	for(;;){
		capture>>rawFrame;
		capture.read(rawFrame);
		undistort(rawFrame, frame_new, cameraMatrix, distCoeffs);
//	for(int i=0;i<5;i++){
//		cout << distCoeffs.at<double>(i)<<" ";
//	}
//	cout<<endl;
//	cout<<endl;
//	for (int i=300; i<305;i++){
//		cout << (int)rawFrame.at<uchar>(300,i) << " " << (int)rawFrame.at<uchar>(301,i) << " " << (int)rawFrame.at<uchar>(302,i) << " " << (int)rawFrame.at<uchar>(303,i) << " " << (int)rawFrame.at<uchar>(304,i) << endl;
//	}

// cout<<rawFrame.rows<<" "<< rawFrame.cols<<endl;



		imshow("Camera Stream", frame_new);
		if(waitKey(30)>=0)
			break;
	}
    return 0;
}
