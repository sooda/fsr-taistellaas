#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

void colorize( int, void* );

Mat src;
Mat dst;

int h = 10;
int s = 50;
int v = 90;

char* win = "window";

int main(int argc, char** argv)
{

	src = imread ( argv[1], 1 );
	cvtColor(src, dst, CV_BGR2HSV);

	imshow("src",src);
//	imshow("dst",dst);


	/* Create Window and Trackbar */
	namedWindow( win, CV_WINDOW_AUTOSIZE );
	createTrackbar( " H: (0-h)", win, &h, 255, colorize );
	createTrackbar( " S: (s-255)", win, &s, 255, colorize );
	createTrackbar( " V: (v-255)", win, &v, 255, colorize );
	colorize( 0, 0 );

	waitKey(0);
	return 0;

}

void colorize (int, void*)
{

//        Mat src, dst;

//	src = imread ( argv[1], 1 );
	cvtColor(src, dst, CV_BGR2HSV);

	// tresholding
//	inRange(dst, Scalar(0, 160, 160), Scalar(10, 255, 255), dst);
	inRange(dst, Scalar(0, s, v), Scalar(h, 255, 255), dst);
/*
	// dilation
	int dilation_size = 2;
	Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
//	dilate( dst, dst, element );

	// contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(dst, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for(int idx = 0; idx >= 0; idx = hierarchy[idx][0] )
	{
//		double area = contourArea(contours[idx]);
		Scalar color( rand()&255, rand()&255, rand()&255 );
//		if (area < 100) continue;
//		if (area > 1000) color = Scalar(0,0,255);

		Point2f center;
		float radius;
		minEnclosingCircle(contours[idx], center, radius);

		if (radius < 15) continue;
		std::cout << idx << ": " << center << " " << radius << std::endl;

//		drawContours( src, contours, idx, color, CV_FILLED, 8, hierarchy );
		circle( src, center, (int)radius * 1.2, color, 2);
	}
*/
//	imshow( "origin", src);
//        imshow( "padam", dst);
	imshow(win, dst);

//	waitKey(0);


}

