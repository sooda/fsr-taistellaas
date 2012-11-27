#ifndef _J2B2_CAMERA_HPP_
#define _J2B2_CAMERA_HPP_

#include <iostream>

#include "MachineCtrlClient.hpp"
#include "../J2B2-API.hpp"
#include "../SLAM/SLAMutil.hpp"
#include "../motion/ServoControl.hpp"
#include "Camutil.hpp"

#define dPan Motion::ServoControl::KServoCameraPTUPan
#define dTilt Motion::ServoControl::KServoCameraPTUTilt

#define FOV 66.58 // degrees = 1.1621 rads

/*
 * Camera and object recognition module
 * Toni Liski / 2012-09-28
*/

namespace cam {

class Camera {

public:

	enum Exception {
		ERR_CAMERA_CLIENT_INITIALIZATION,
		ERR_GET_IMAGE_DATA,
		ERR_GET_IMAGE
	};

	// constructor initialises the Camera module
	// takes CImageClient and ServoPosition as parameter
	Camera(CJ2B2Client&);

	// destructor
	~Camera();

	// updates data from the camera and recognise objects
	void updateCameraData(SLAM::RobotLocation);
	MaCI::Image::CImageContainer getCameraImage();

	// Calibrate camera
	bool calibrateCamera();

	std::vector<Location> getPositionOfTargets();


private:
	CJ2B2Client &interface;

	// image data
	struct cameradata_t {
		MaCI::Image::CImageContainer cameraImage;
		float tilt;
		float pan;
		SLAM::RobotLocation robotloc;
	} cameradata;

	// Position of the balls in a real world
	std::vector<Location> balls;

	bool calibrated;
	bool show_image;

	Motion::ServoControl servoCtrl;

	
	
	void getCameraData();
	void checkCalibration();
	bool ballsInImage();
	
	// TODO: add some functions to recognise objects

	Eigen::Matrix3f getObjectRotation(const float, const float);
	void updateCamServos();
	void updatePositionOfTargets();
	
};

}

#endif
