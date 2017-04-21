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

	// -------------------- Capture frame from video ----------------------------
	Mat frame, frame_new;

	namedWindow("Camera Window", 1);
	for(;;){
		capture>>frame;
		capture.read(frame);
		undistort(frame, frame_new, cameraMatrix, distCoeffs);
		imshow("Camera Stream", frame_new);
		if(waitKey(30)>=0)
			break;
	}
    return 0;
}
