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

class Location {

public:
	// location in real world or object in an image

	Location(double x_val, double y_val) 
		: x(x_val), y(y_val) { }

	double x;
	double y;

};

class Camutil {

public:

	// constructor
	Camutil();

	static cv::Mat imgToMat (MaCI::Image::CImageContainer);
	static bool BallsInView (cv::Mat);
	static std::vector<Location> FindNonTargets (cv::Mat);
	static std::vector<Location> FindBalls (cv::Mat);
	static std::vector<Location> FindBalls (cv::Mat, bool show_image);
	static std::vector<Location> FindBalls (cv::Mat, bool show_image, bool targets);

private:


};
}
#endif
