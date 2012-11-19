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
using namespace cv;

namespace cam {

Camera::Camera(CJ2B2Client &interface)
	: interface(interface), cameraImage(CImageContainer()), calibrated(false),
	  show_image(false), servoCtrl(interface)
{
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

	// show image in separate window
	if (show_image) {
		Mat src = Camutil::imgToMat(this->getCameraImage());
		Camutil::FindBalls(src);
	}
}

bool Camera::ballsInImage() {
	Mat src = Camutil::imgToMat(this->getCameraImage());
	return Camutil::FindBalls(src);
}

void Camera::getCameraData()
{
	bool r;
	unsigned int imgSeq = 0;

	MaCI::Image::CImageData imgData;

	if (!interface.iImageCameraFront) {
		throw ERR_CAMERA_CLIENT_INITIALIZATION;
	}

	r = interface.iImageCameraFront->GetImageData(imgData, &imgSeq, 0);
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

CImageContainer Camera::getCameraImage()
{
	return this->cameraImage;
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
