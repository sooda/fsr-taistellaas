#include <iostream>

#include "Camutil.hpp"

/*
 * Camera Utility class
 * Toni Liski / 2012-10-17
*/

using namespace cv;
using namespace Eigen;

namespace cam {

Camutil::Camutil() {
	// empty constructor
}

Mat Camutil::imgToMat (MaCI::Image::CImageContainer srcimg)
{
	const MaCI::Image::TImageInfo imginfo = srcimg.GetImageInfoRef();
	const unsigned int rows = imginfo.imageheight;
	const unsigned int cols = imginfo.imagewidth;

	Mat img = Mat(rows, cols, CV_8UC3, (unsigned char*)srcimg.GetImageDataPtr());
	return img;
}

bool Camutil::BallsInView (Mat src)
{
	std::vector<Location> object = FindBalls(src, false);
	return !object.empty();
}

std::vector<Location> Camutil::FindNonTargets (Mat src) {
	return FindBalls (src, false, false);
}

std::vector<Location> Camutil::FindBalls (Mat src) {
	return FindBalls (src, false, true);
}

std::vector<Location> Camutil::FindBalls (Mat src, bool show_image) {
	return FindBalls (src, show_image, true);
}

std::vector<Location> Camutil::FindBalls (Mat src, bool show_image, bool targets)
{
	int limit_h = 0;
	int limit_s = 0;
	int limit_v = 0;

	if (targets) {
		limit_h = 10;
		limit_s = 50;
		limit_v = 90;
	}
	else {
		// TODO: !!
		limit_h = 10;
		limit_s = 50;
		limit_v = 90;	}
	
	Mat dst;
	cvtColor(src, dst, CV_BGR2HSV);
	std::vector<Location> objects;

	// tresholding
	inRange(dst, Scalar(0, limit_s, limit_v), Scalar(limit_h, 255, 255), dst);

	// dilation
	int dilation_size = 2;
	Mat element = getStructuringElement( MORPH_RECT,
					Size( 2*dilation_size + 1, 2*dilation_size + 1 ),
					Point( dilation_size, dilation_size ) );
	dilate( dst, dst, element );

	// contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(dst, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for(int idx = 0; idx >= 0; idx = hierarchy[idx][0] )
	{
		Scalar color( rand()&255, rand()&255, rand()&255 ); // random color

		Point2f center;
		float radius;
		minEnclosingCircle(contours[idx], center, radius);

		if (radius < 15) continue;
		std::cout << idx << ": " << center << " " << radius << std::endl;

		circle( src, center, (int)radius * 1.2, color, 2);
		
		objects.push_back(Location(center.x, center.y));
	}

	if (show_image)
		imshow( "padam", dst);

//	waitKey(0);

	return objects;

}

}
