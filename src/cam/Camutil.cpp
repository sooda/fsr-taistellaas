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
	Mat img = Mat(rows, cols, CV_8UC1);
	
	if (srcimg.GetImageDataType() != MaCI::Image::EImageDataType::KImageDataJPEG) {
		std::cout << "ImgDataType not JPEG!!!" << std::endl;;
		return img;
	}

	bool success = srcimg.ConvertTo(MaCI::Image::EImageDataType::KImageDataRGB);
	if (success) {
		img = Mat(rows, cols, CV_8UC3, (unsigned char*)srcimg.GetImageDataPtr());
	}
	else {
		std::cout << "Error converting image to Mat" << std::endl;
	}

	return img.clone();
}

bool Camutil::BallsInView (Mat src)
{
	std::vector<SLAM::Location> object = FindBalls(src, false);
	return !object.empty();
}

std::vector<SLAM::Location> Camutil::FindNonTargets (Mat src) {
	return FindBalls (src, false, false);
}

std::vector<SLAM::Location> Camutil::FindBalls (Mat src) {
	return FindBalls (src, false, true);
}

std::vector<SLAM::Location> Camutil::FindBalls (Mat src, bool show_image) {
	return FindBalls (src, show_image, true);
}

std::vector<SLAM::Location> Camutil::FindBalls (Mat src, bool show_image, bool targets)
{

	std::vector<SLAM::Location> objects;

	int limit_h_max = 0;
	int limit_h_min = 0;
	int limit_s = 0;
	int limit_v = 0;

	if (targets) {
		limit_h_min = 0;
		limit_h_max = 10;
		limit_s = 50;
		limit_v = 90;
	}
	else {
		limit_h_min = 45;
		limit_h_max = 70;
		limit_s = 120;
		limit_v = 120;
	}
	
	Mat dst;

	cvtColor(src, dst, CV_RGB2HSV);

	namedWindow("src", CV_WINDOW_AUTOSIZE);

	// tresholding
	inRange(dst, Scalar(limit_h_min, limit_s, limit_v), Scalar(limit_h_max, 255, 255), dst);

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

	for( int idx = 0; idx < contours.size(); idx++ )
	{
		//Scalar color( rand()&255, rand()&255, rand()&255 ); // random color
		Scalar color(0,0,255);
		if (!targets) { color = Scalar(0,255,0); }
		
		Point2f center;
		float radius;
		minEnclosingCircle(contours[idx], center, radius);

		if (radius < 13) continue;
//		std::cout << idx << ": " << center << " " << radius << std::endl;

		circle( src, center, (int)radius * 1.2, color, 2);
		
		objects.push_back(SLAM::Location(center.x, center.y));
	}
	
	if (show_image) {
		imshow( "src", src);
		waitKey(10);
	}

	return objects;

}

}
