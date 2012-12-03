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
	: interface(interface), calibrated(false),
	  show_image(true), servoCtrl(interface)
{
	servoCtrl.TestMovement();
	lastrun = ownTime_get_ms();
}

void Camera::updateCameraData(SLAM::RobotLocation robloc)
{
	cameradata.robotloc = robloc;
	this->updateCamServos();
	this->checkCalibration();
	this->getCameraData();

	// find balls from image only when updating SLAM
	// this->updatePositionOfTargets();
	
}

bool Camera::ballsInImage() {
	Mat src = Camutil::imgToMat(this->getCameraImage());
	return Camutil::BallsInView(src);
}

void Camera::getCameraData()
{
	bool r = false;
	unsigned int imgSeq = 0;

	MaCI::Image::CImageData imgData;

	if (!interface.iImageCameraFront) {
		cout << "error 1" << endl;
		throw ERR_CAMERA_CLIENT_INITIALIZATION;
	}

	r = interface.iImageCameraFront->GetImageData(imgData, &imgSeq);
	if (!r) {
		cout << "error 2" << endl;
		throw ERR_GET_IMAGE_DATA;
	}

	// Lock();
	r = imgData.GetImage(cameradata.cameraImage, NULL, true);
	// Unlock();

	if (!r) {
		cout << "error 3" << endl;
		throw ERR_GET_IMAGE;
	}

}

void Camera::updateToSLAM(SLAM::SLAM &slam) {

	if (ownTime_get_ms_since(lastrun) < 1000) {
		return;
	}

	lastrun = ownTime_get_ms();
	updatePositionOfTargets();

	std::cout << "Cam: Update SLAM data" << std::endl;

	SLAM::RobotLocation location = cameradata.robotloc; // location image was taken
	double minDist = 0.5; // image front in m
	double maxDist = 3; // image back in m
	double viewWidth = M_PI*FOV/180; // image fov in rad

	for (int type_ = SLAM::MapData::TARGET; type_ != SLAM::MapData::OBS_TYPE_SIZE; type_++) {

		SLAM::MapData::ObservationType type = (SLAM::MapData::ObservationType)type_;
		std::vector<SLAM::Location> objects;

		if (type == SLAM::MapData::TARGET) {
			objects = balls; // target locations in (dx,dy) from location robot was when image was taken
		}
		else if (type == SLAM::MapData::OBSTACLE) {
			objects= nontargets; // as above
		}
		else if (type == SLAM::MapData::GOAL) {
			// daa
		}

		SLAM::ImageData data (objects, location, minDist, maxDist, viewWidth);

		slam.updateImageData(data, type);

	}




}

CImageContainer Camera::getCameraImage()
{
	return this->cameradata.cameraImage;
}

void Camera::checkCalibration()
{
	if (!this->calibrated) calibrateCamera();
}

bool Camera::calibrateCamera()
{
	// TODO: implement or remove
	return (calibrated = true);
}

void Camera::updateCamServos()
{
	// TODO: not working!
	cameradata.tilt = servoCtrl.getPosition(dTilt);
	cameradata.pan  = servoCtrl.getPosition(dPan);
}

Eigen::Matrix3f Camera::getObjectRotation(const float left, const float top)
{
    // image
    const MaCI::Image::TImageInfo imginfo = cameradata.cameraImage.GetImageInfoRef();
    const float height = (float)imginfo.imageheight;
    const float width = (float)imginfo.imagewidth;

//    const float width = 640;
//    const float height = 480;

    // servos
    const float tilt = cameradata.tilt;
    const float pan = cameradata.pan;

    const float fov = M_PI*FOV/180;

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

void Camera::updatePositionOfTargets()
{

	balls.clear();
/*
	if (ownTime_get_ms_since(lastrun) < 1000) {
		return;
	}
	lastrun = ownTime_get_ms();
*/

	// find the objects in the current camera data
	std::vector<SLAM::Location> objects = Camutil::FindBalls(Camutil::imgToMat(this->getCameraImage()), show_image);
	
	for (size_t i = 0; i < objects.size(); i++) {

		SLAM::Location ball = objects.at(i);
	
		Matrix3f R = getObjectRotation(ball.x, ball.y);

		Vector3f T(0, 0, 1); // floor
		Vector3f p(0.17, 0, 0.76); // position of the camera
		Vector3f v(1, 0, 0); // direction of the view of the camera
		Vector3f t(0,0,0);

		v = R.transpose() * v;

		t = p - ((T.transpose() * p)/(T.transpose() * v) * v.transpose()).transpose();

		double theta = cameradata.robotloc.theta;
		double x = t.x() * cos(theta) + t.y() * sin(theta);
		double y = t.x() * sin(theta) - t.y() * sin(theta);

		// TODO: CHECK THE CALCULATIONS AND REMOVE THESE LINES!
		if (x > 3) x = 3;
		if (y > 3) y = 3;

//		cout << "pallo: " << t << endl;
		balls.push_back(SLAM::Location(x, y));

		float distance = sqrt(t.x() * t.x() + t.y() * t.y());
		cout << "Cam: Target at (" << x << ", " << y << ") " << "dist: " << distance << endl;

	}
}

std::vector<SLAM::Location> Camera::getPositionOfTargets()
{
	return balls;
}


}
