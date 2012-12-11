#ifndef _J2B2_CAMERA_HPP_
#define _J2B2_CAMERA_HPP_

#include <iostream>

#include "MachineCtrlClient.hpp"
#include "J2B2-API.hpp"
#include "SLAM/includes.hpp"
#include "motion/ServoControl.hpp"
#include "cam/Camutil.hpp"
#include "cam/CameraCalibration.hpp"

#define dPan Motion::ServoControl::KServoCameraPTUPan
#define dTilt Motion::ServoControl::KServoCameraPTUTilt

// TODO: oiskoha kuitenki 54?
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

	Camera(CJ2B2Client&, Motion::ServoControl&);

	// updates data from the camera and recognise objects
	void updateCameraData(SLAM::RobotLocation);
	MaCI::Image::CImageContainer getCameraImage();

	// Calibrate camera
	bool calibrateCamera();

	// update camera data to SLAM
	void updateToSLAM(SLAM::SLAM &slam);


private:
	bool calibrated;
	bool show_image;

	Motion::ServoControl& servoCtrl;
	CJ2B2Client& interface;

	// image data
	struct cameradata_t {
		MaCI::Image::CImageContainer cameraImage;
		float tilt;
		float pan;
		SLAM::RobotLocation robotloc;
	} cameradata;

	// Position of the balls in a real world
	std::vector<SLAM::Location> balls;
	std::vector<SLAM::Location> nontargets;
	std::vector<SLAM::Location> goalarea;

	ownTime_ms_t lastrun;

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	
	void getCameraData();
	void checkCalibration();
	bool ballsInImage();
	
	void updateCamServos();
	void updatePositionOfTargets();
	
};

}

#endif
