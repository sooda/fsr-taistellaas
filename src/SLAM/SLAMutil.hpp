/*

Utility stuff and such

*/

#ifndef _J2B2_UTILITIES_HPP_
#define _J2B2_UTILITIES_HPP_

namespace SLAM {

class RobotLocation {

public:

	RobotLocation(double x_val, double y_val, double theta_val) 
		: x(x_val), y(y_val), theta(theta_val) { }

	double x; 		// in meters
	double y; 		// in meters
	double theta;	// in radians

};

class Location {

public:

	Location(double x_val, double y_val) 
		: x(x_val), y(y_val) { }

	double x;	// in meters
	double y;	// in meters

};

class GridPoint {

public:

	GridPoint(int x_val, int y_val) 
		: x(x_val), y(y_val) { }

	int x;
	int y;

};

}

#endif

