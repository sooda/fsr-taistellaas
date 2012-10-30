#ifndef _J2B2_CAMUTIL_HPP_
#define _J2B2_CAMUTIL_HPP_

#include "MachineCtrlClient.hpp"
#include "../J2B2-API.hpp"
#include "../SLAM/SLAMutil.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

/*
 * Camera Utility class
 * Toni Liski / 2012-10-17
*/

namespace cam {

class Camutil {

public:

	// constructor
	Camutil();

	static cv::Mat imgToMat (MaCI::Image::CImageContainer);
	static void FindBalls (cv::Mat);

private:


};
}
#endif
