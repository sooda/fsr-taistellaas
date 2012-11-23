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
using namespace Eigen;
using namespace std;

namespace cam {

Camera::Camera(MaCI::MachineCtrl::CMachineCtrlClient *machine, Motion::ServoControl& servos)
	: machineCtrl(machine), cameraClient(NULL),
	  cameraImage(CImageContainer()), calibrated(false), show_image(true),
	  servoCtrl(servos)
{
	// Get component list
	MaCI::MachineCtrl::TMaCIMachineComponents comp;
	machineCtrl->GetMachineComponents(comp);

	// If the list has any components ...
	if (comp.size() > 0) {
		cameraClient = machineCtrl->GetImageClient("CameraFront", true);
	}
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

	MaCI::Image::CImageData imgData;

	if (!this->cameraClient) {
		throw ERR_CAMERA_CLIENT_INITIALIZATION;
	}

	r = this->cameraClient->GetImageData(imgData, &imgSeq, 0);
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
	string source_window = "Source";
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

	for (size_t i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
//		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		minEnclosingCircle(contours_poly[i], center[i], radius[i]);
	}

	/// Draw polygonal contour + bonding rects + circles
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (size_t i = 0; i < contours.size(); i++) {
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

Eigen::Vector2f Camera::getCamRotation()
{
	float tilt = servoCtrl.getPosition(dTilt);
	float pan = servoCtrl.getPosition(dPan);

	Vector2f angles (tilt, pan);

	return angles;

}

Eigen::Matrix3f Camera::getObjectRotation(const float left, const float top)
{
	// image
    const float width = 640;
    const float height = 480;

    // servos
	Vector2f angles = getCamRotation();
    const float tilt = angles(0);
    const float pan = angles(1);

    const float fov = FOV / 360.0 * M_PI;

    // yksi pikseli vastaa radiaaneja (pystysuunnassa) (pixel per degree) (voidaanko käyttää myös vaakasuunnassa?)
    const float ppd = fov/height;

    // kohteen etäisyys keskipisteestä
    float ydist = height/2 - top;
    float xdist = width/2 - left;

    // tarkoittaa kulmina xangl radiaania
    float yangl = ppd*ydist;
    float xangl = ppd*xdist;

    // tällöin kulma
    float ya = tilt + yangl;
    float xa = pan + xangl;


	Matrix3f rotation;

    rotation = AngleAxisf(ya, Vector3f::UnitY()) * AngleAxisf(xa, Vector3f::UnitZ());

	return rotation;

}

void Camera::getPositionOfTargets()
{
    // object
    const int left = 320;
    const int top = 240;

	Matrix3f R = getObjectRotation(left, top);

	Vector3f T(0, 0, 1); // floor
    Vector3f p(5, 0, 50); // position of the camera
    Vector3f v(1, 0, 0); // direction of the view of the camera
    Vector3f t(0,0,0);

    v = R.transpose() * v;
    cout << v << endl;

    t = p - ((T.transpose() * p)/(T.transpose() * v) * v.transpose()).transpose();

    cout << "pallo: " << t << endl;

    float distance = sqrt(t.x() * t.x() + t.y() * t.y());

    cout << "dist: " << distance << endl;



}

}
