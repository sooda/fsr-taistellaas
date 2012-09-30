/*
 * Camera.cpp
 *
 *  Created on: 30 Sep 2012
 *      Author: Toni Liski
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "Camera.hpp"

using namespace MaCI::Image;

namespace cam {

Camera::Camera(MaCI::Image::CImageClient cameraClient,
	const ServoPosition servoPosition)
//	: cameraClient(cameraClient), servoPosition(servoPostion)
{
	calibrated = false;
	show_image = true;
}

Camera::~Camera()
{
	// empty
}

Location* Camera::getDistanceToObjects()
{
	return new Location();
}

void Camera::updateCameraData()
{
	this->checkCalibration();
	this->getCameraData();
	// process data from camera
	this->showImage();
}

void Camera::getCameraData()
{
	bool r;
	unsigned int imgSeq = 0;

	if (!this->cameraClient) {
		throw ERR_CAMERA_CLIENT_INITIALIZATION;
	}

	r = this->cameraClient->GetImageData(this->imgData, &imgSeq, 0);
	if (r) {
		throw ERR_GET_IMAGE_DATA;
	}

	// Lock();
	r = imgData.GetImage(cameraImage, NULL, true);
	// Unlock();

	if (!r) {
		throw ERR_GET_IMAGE;
	}

}

void Camera::checkCalibration()
{
	if (!this->calibrated) calibrateCamera();
}

bool Camera::calibrateCamera()
{
	// TODO: implement
	return (calibrated = true);
}
void Camera::showImage()
{
	using namespace cv;
	using namespace std;

	Mat src; Mat src_gray;
	int thresh = 100;
	RNG rng(12345);

	/// Load source image and convert it to gray
	src = imread( "floorball.jpg", 1 );

	/// Convert image to gray and blur it
	cvtColor( src, src_gray, CV_BGR2GRAY );
	blur( src_gray, src_gray, Size(3,3) );

	/// Create Window
	char* source_window = "Source";
	namedWindow( source_window, CV_WINDOW_AUTOSIZE );
	imshow( source_window, src );

	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using Threshold
	threshold(src_gray, threshold_output, thresh, 255, THRESH_BINARY);
	/// Find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly(contours.size());
//	vector<Rect> boundRect(contours.size());
	vector<Point2f> center(contours.size());
	vector<float> radius(contours.size());

	for (int i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
//		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		minEnclosingCircle(contours_poly[i], center[i], radius[i]);
	}

	/// Draw polygonal contour + bonding rects + circles
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i> (), 0, Point());

		// rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );

		if (radius[i] < 20) continue;
		circle(drawing, center[i], (int) radius[i], color, 2, 8, 0);
	}

	/// Show in a window
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("Contours", drawing);
}

}
