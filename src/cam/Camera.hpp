#ifndef _J2B2_CAMERA_HPP_
#define _J2B2_CAMERA_HPP_

#include <MaCI/MachineCtrlClient.hpp>
#include "../J2B2-API.hpp"
#include "../SLAM/SLAMutil.hpp"

/*
 * Camera and object recognition module
 * Toni Liski / 2012-09-28
*/

namespace cam {

class ServoPosition { };
class Location { };

class Camera {

public:

	enum Exception {
		ERR_CAMERA_CLIENT_INITIALIZATION,
		ERR_GET_IMAGE_DATA,
		ERR_GET_IMAGE
	} exp;

	// constructor initializes the Camera module
	// takes CImageClient and ServoPosition as parameter
	Camera(MaCI::Image::CImageClient, const ServoPosition);

	// copy constructor
	Camera(const cam::Camera&);
	Camera operator=(const Camera&);

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

	bool calibrated;
	bool show_image;

	MaCI::Image::CImageClient *cameraClient;
	MaCI::Image::CImageData imgData;
	MaCI::Image::CImageContainer cameraImage;

	// distances
	MaCI::Ranging::TDistance lastDistance;
	MaCI::Ranging::TDistanceArray lastLaserDistanceArray;

	// ServoPosition contains information about camera servos
	const ServoPosition *servoPosition;


	void getCameraData();
	void checkCalibration();

	// TODO: add some functions to recognize objects

	void showImage();
};

}

#endif
