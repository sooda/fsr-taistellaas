#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

using namespace cv;
using namespace Eigen;
using namespace std;

int main(int argc, char** argv)
{
	Mat src;
	Mat dst;

	src = imread ( argv[1], 1 );
	cvtColor(src, dst, CV_BGR2HSV);

	
	namedWindow("src", CV_WINDOW_AUTOSIZE);

	// tresholding
	inRange(dst, Scalar(90, 120, 120), Scalar(130, 255, 255), dst);

	// dilation
	int dilation_size = 2;
	Mat element = getStructuringElement( MORPH_RECT,
					Size( 2*dilation_size + 1, 2*dilation_size + 1 ),
					Point( dilation_size, dilation_size ) );
	dilate( dst, dst, element );

	// contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(dst, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	vector<vector<Point> > contours_poly( contours.size() );
  
	for( int i = 0; i < contours.size(); i++ )
	{
		Scalar color(255, 0, 0);

		approxPolyDP( Mat(contours[i]), contours_poly[i], arcLength(Mat(contours[i]), true)*0.02, true );		
		
		drawContours( src, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		drawContours( src, contours_poly, i, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
		
		std::cout << contours_poly[i] << std::endl;
		
		for (int a = 0; a < contours_poly[i].size(); a++) {
			circle(src, contours_poly[i][a], 10, Scalar(0,0,255));
		}
		
	}
	
	imshow( "src", src);
	waitKey(0);

	return 0;
}

