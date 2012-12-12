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


#define ANGLE_FAR -0.8
#define MAX_DIST_FAR 1.8
#define MIN_DIST_FAR 0.5

#define ANGLE_NEAR -1.25
#define MAX_DIST_NEAR 0.9
#define MIN_DIST_NEAR 0.2

namespace cam {

Camera::Camera(CJ2B2Client& interface, Motion::ServoControl& servos)
	: calibrated(true), show_image(true), servoCtrl(servos), interface(interface)
{
	cameradata.initd = false;
	servoCtrl.resetServos();
	rotateFar();
	lastrun = ownTime_get_ms();
	cameraRotated = ownTime_get_ms();
	
	this->cameraMatrix = (Mat_<double>(3,3) << 447.7679638984855, 0, 309.9314941852911, 0, 442.5677667837122, 245.2180309042916, 0, 0, 1);
	this->distCoeffs   = (Mat_<double>(1,5) << 0.08322752073634829, -0.2949847441859801, 0.002063609323629203, 0.0005587292612777254, 0.3479390918780765);

}

void Camera::updateCameraData(SLAM::RobotLocation robloc)
{
	cameradata.robotloc = robloc;
	this->updateCamServos();
	this->checkCalibration();
	this->getCameraData(); // could be moved before?

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
	cameradata.initd = true;
}

void Camera::updateToSLAM(SLAM::SLAM &slam)
{
	if (!cameradata.initd)
		return;

	int waitTime = 100;

	if (ownTime_get_ms_since(lastrun) < waitTime) {
		return;
	}
	
	// if we have just rotated camera, skip
	if (ownTime_get_ms_since(cameraRotated) < 500) {
		return;
	}
	
	lastrun = ownTime_get_ms();
	updatePositionOfTargets();

	std::cout << "Cam: Update SLAM data" << std::endl;

	SLAM::RobotLocation location = cameradata.robotloc; // location image was taken

	// far position
	double minDist = MIN_DIST_FAR; // image front in m
	double maxDist = MAX_DIST_FAR; // image back in m
	if (fabs(cameradata.tilt-ANGLE_NEAR) < fabs(cameradata.tilt-ANGLE_FAR)) {
		// near position
		minDist = MIN_DIST_NEAR; // image front in m
		maxDist = MAX_DIST_NEAR; // image back in m
	}
	
	// TODO: Correct FOV??
	double viewWidth = M_PI*FOV/180; // image fov in rad 

	for (int type_ = SLAM::MapData::TARGET; type_ != SLAM::MapData::OBS_TYPE_SIZE; type_++) {

		SLAM::MapData::ObservationType type = (SLAM::MapData::ObservationType)type_;
		std::vector<SLAM::Location> objects;

		if (type == SLAM::MapData::TARGET) {
			objects = balls;
		}
		else if (type == SLAM::MapData::OBSTACLE) {
			objects= nontargets;
		}
		else if (type == SLAM::MapData::GOAL) {
			objects = goalarea;
		}

		SLAM::ImageData data (objects, location, minDist, maxDist, viewWidth);

		slam.updateImageData(data, type);

	}

}

CImageContainer Camera::getCameraImage()
{
	return this->cameradata.cameraImage;
}

void Camera::rotateNear() {
	cameraRotated = ownTime_get_ms();
	servoCtrl.setPosition(dTilt, ANGLE_NEAR);
	servoCtrl.setPosition(dPan, 0.0);
}

void Camera::rotateFar() {
	cameraRotated = ownTime_get_ms();
	servoCtrl.setPosition(dTilt, ANGLE_FAR);
	servoCtrl.setPosition(dPan, 0.0);
}

bool Camera::isRotatedNear() const {
	return this->isRotatedNear(servoCtrl.getPosition(dTilt));
}

bool Camera::isRotatedNear(float camPos) const {
	return (fabs(camPos-ANGLE_NEAR) < fabs(camPos-ANGLE_FAR));
}

void Camera::checkCalibration()
{
	// enable camera calibration by changing calibrated variable in the instruction list
	if (!this->calibrated) calibrateCamera();
}

bool Camera::calibrateCamera()
{
	CameraCalibration calibration(interface);
	calibration.runCalibration();

	return (calibrated = true);
}

void Camera::updateCamServos()
{
	// TODO: not working!
	cameradata.tilt = servoCtrl.getPosition(dTilt);
	cameradata.pan  = servoCtrl.getPosition(dPan);
}

Matrix3f rotx(double alpha) {
	double c = cos(alpha), s = sin(alpha);
	Matrix3f ret;
	ret <<
			1, 0, 0,
			0, c, -s,
			0, s, c;
	return ret;
}
Matrix3f roty(double alpha) {
	double c = cos(alpha), s = sin(alpha);
	Matrix3f ret;
	ret <<
			c, 0, s,
			0, 1, 0,
			-s, 0, c;
	return ret;
}
Matrix3f rotz(double alpha) {
	double c = cos(alpha), s = sin(alpha);
	Matrix3f ret;
	ret <<
			c, -s, 0,
			s, c, 0,
			0, 0, 1;
	return ret;
}

void Camera::updatePositionOfTargets()
{
	this->balls.clear();
	this->nontargets.clear();
	this->goalarea.clear();

	// find the objects in the current camera data

	Mat image = Camutil::imgToMat(this->getCameraImage());
	Mat temp = image;
//	undistort(image, temp, this->cameraMatrix, this->distCoeffs);

	Mat drawing = temp.clone();
	
	std::vector<SLAM::Location> targets = Camutil::FindBalls(temp, drawing);
	std::vector<SLAM::Location> non_targets = Camutil::FindBalls(temp, drawing, false);
	std::vector<SLAM::Location> goal_area = Camutil::FindGoalArea(temp, drawing);

	imshow("targets", drawing);
	waitKey(10); // needed?
	
	for (size_t type = 0; type <= 2; type++)
	{
		std::vector<SLAM::Location> objects;
		
		if (type == 0)
			objects = targets;
		else if (type == 1)
			objects = non_targets;
		else if (type == 2)
			objects = goal_area;
			
		for (size_t i = 0; i < objects.size(); i++) {

			SLAM::Location ball = objects.at(i);
			
			double theta_cam = cameradata.tilt - 0.07; // TODO: correct the magic constant to a good place
			double theta_rob = -cameradata.robotloc.theta;
			double f = 0.0037; // as it reads in the lens
			double pxlsz = 0.000007; // hacked experimentally, physical pixel size on image sensor
			double m = f / pxlsz;

			Matrix3f K;
			K << m, 0, 320, 0, m, 240, 0, 0, 1;
			Matrix3f Ki = K.inverse();

			Vector3f P_cam(0, 0.72, -0.27);
			Vector3f P_rob(cameradata.robotloc.y, 0, -cameradata.robotloc.x); // minuses?
			Matrix3f R_cam(rotx(theta_cam));
			Matrix3f R_rob(roty(theta_rob));
			Vector3f p(ball.x, ball.y, 1);
			Vector3f v = R_rob * R_cam * Ki * p;
			double l = -P_cam(1) / v(1);
			Vector3f P0 = P_rob + R_rob * P_cam;
			Vector3f P = P0 + l * v;
			Vector3f t(-P(2), P(0), 0);

			double dx = t.x() - cameradata.robotloc.x;
			double dy = t.y() - cameradata.robotloc.y;

			double r = sqrt(dx*dx + dy*dy);
			double theta = atan2(dy, dx) - cameradata.robotloc.theta;
			theta = atan2(sin(theta), cos(theta));
			
			cout << "Cam: " << (type==0?"Target":type==1?"nontarget":"goalarea") << " at (" << t.x() << ", " << t.y() << ") " << " = (r " << r << ", t " << theta << ")" << endl;

			/*double minDist = MIN_DIST_FAR; // image front in m
			double maxDist = MAX_DIST_FAR; // image back in m
			if (this->isRotatedNear(cameradata.tilt)) {
				// near position
				minDist = MIN_DIST_NEAR; // image front in m
				maxDist = MAX_DIST_NEAR; // image back in m
			}

			if ((distance > maxDist) || (distance < minDist)) {
				std::cout << "Cam: ball omitted" << std::endl;
				continue;
			}*/

			if (type == 0)
				this->balls.push_back(SLAM::Location(t.x(), t.y()));
			else if (type == 1)
				this->nontargets.push_back(SLAM::Location(t.x(), t.y()));
			else if (type == 2)
				this->goalarea.push_back(SLAM::Location(t.x(), t.y()));

		}
	}
}

}
