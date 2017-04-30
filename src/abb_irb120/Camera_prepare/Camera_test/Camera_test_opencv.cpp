// OpenCV to see the camera.

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>



using namespace cv;
int main(){
	VideoCapture capture(0);
	if(!capture.isOpened())
		return -1;
	Mat frame;
	namedWindow("Camera Window", 1);
	for(;;){
		capture>>frame;
		capture.read(frame);
		imshow("Camera Stream", frame);
		if(waitKey(30)>=0)
			break;
	}
	return 0;
}
