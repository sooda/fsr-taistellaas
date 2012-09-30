#ifndef _J2B2_CAMERA_HPP_
#define _J2B2_CAMERA_HPP_

#include "MachineCtrlClient.hpp"
#include "../SLAM/SLAMutil.hpp"

/*
 * Camera and object recognition module
 * Toni Liski / 2012-09-28
*/

namespace cam {

class Camera {

public:

	// constructor initializes the Camera module
	// takes CImageClient and ServoPosition as parameter
	Camera(MaCI::Image::CImageClient *cameraClient,
		ServoPosition *servoPosition);

	// destructor
	~Camera();

	// returns the array of location of the detected objects
	// TODO: let's make a class for locations instead of array
	Location* getDistanceToObjects();

	// updates data from the camera and recognise objects
	void updateCameraData();

	// Calibrate camera
	bool calibrateCamera();

private:

	bool calibrated = false;

	MaCI::Image::CImageClient *cameraClient;
	MaCI::Image::CImageData imgData;
	MaCI::Image::CImageContainer cameraImage;

	// distances
	MaCI::Ranging::TDistance lastDistance;
	MaCI::Ranging::TDistanceArray lastLaserDistanceArray;

	// ServoPosition contains information about camera servos
	 ::ServoPosition *servoPosition;


	void GetCameraData();

	// TODO: add some functions to recognize objects

};

}

#endif