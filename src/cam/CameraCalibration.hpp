#ifndef _J2B2_CAMERACALIBRATION_HPP_
#define _J2B2_CAMERACALIBRATION_HPP_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

#include "MachineCtrlClient.hpp"
#include "../J2B2-API.hpp"


namespace cam {

class CameraCalibration {

public: 

	CameraCalibration(CJ2B2Client& j2b2);
	
	cv::Mat getImage();
	void runCalibration();

private:
	CJ2B2Client& j2b2;
	
};

}

#endif
