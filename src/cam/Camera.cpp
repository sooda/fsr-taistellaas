/*
 * Camera.cpp
 *
 *  Created on: 30 Sep 2012
 *      Author: Toni Liski
 */

#include "Camera.hpp"

using namespace MaCI::Image;

Camera::Camera(MaCI::Image::CImageClient *cameraClient,
	ServoPosition *servoPosition)
	: cameraClient(cameraClient), servoPosition(servoPostion)
{

}

Camera::~Camera()
{
	// empty
}

Location* Camera::getDistanceToObjects()
{

}

void Camera::updateCameraData()
{
	this->getCameraData();
}

void Camera::getCameraData()
{
	bool r;
	unsigned int imgSeq = 0;

	if (this->cameraClient) {
		r = this->cameraClient->GetImageData(imgData, &imgSeq);
		if (r) {

		}
		// Lock();
		r = imgData.GetImage(cameraImage, NULL, true);
		// Unlock();

		if (!r) {

		}



	}
}

void Camera::checkCalibration()
{
	if (!this->calibrated) calibrateCamera();
}

bool Camera::calibrateCamera()
{

}

