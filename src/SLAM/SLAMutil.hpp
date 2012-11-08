/*

Utility stuff and such

*/

#ifndef _J2B2_UTILITIES_HPP_
#define _J2B2_UTILITIES_HPP_

#include <cmath>


namespace SLAM {

class RobotLocation {

public:

	RobotLocation(double x_val, double y_val, double theta_val) 
		: x(x_val), y(y_val), theta(theta_val) { }

	RobotLocation(const RobotLocation &other) 
		: x(other.x), y(other.y), theta(other.theta) { }

	RobotLocation() 
		: x(0), y(0), theta(0) { }

	double x; 		// in meters
	double y; 		// in meters
	double theta;	// in radians

	inline void normalizeTheta() {
		theta = atan2(sin(theta), cos(theta));
	}

	inline bool operator==(const RobotLocation &other) const {
		return (x == other.x && y == other.y && theta == other.theta);
	}

	inline RobotLocation operator-(const RobotLocation &other) const {
		return RobotLocation(x-other.x, y-other.y, theta-other.theta);
	}

	inline RobotLocation operator+(const RobotLocation &other) const {
        return RobotLocation(x+other.x, y+other.y, theta+other.theta);
    }

	inline RobotLocation operator-=(const RobotLocation &other) {
		x -= other.x;
		y -= other.y;
		theta -= other.theta;
        return *this;
    }

    inline RobotLocation operator+=(const RobotLocation &other) {
        x += other.x;
        y += other.y;
        theta += other.theta;
		return *this;
    }

	inline RobotLocation operator/(const double scale) const {
        return RobotLocation(x/scale, y/scale, theta);
    }

    inline RobotLocation operator*(const double scale) const {
        return RobotLocation(x*scale, y*scale, theta);
    }

    inline RobotLocation operator/=(const double scale) {
        x /= scale;
        y /= scale;
        return *this;
    }

    inline RobotLocation operator*=(const double scale) {
        x *= scale;
        y *= scale;
        return *this;
    }

	inline RobotLocation operator=(const RobotLocation &other) {
		if (this == &other) return *this;
		x = other.x;
		y = other.y;
		theta = other.theta;
		return *this;
    }

	// rotation
	inline RobotLocation operator%(const double angle) const {
        return RobotLocation(x*cos(angle)-y*sin(angle), 
		                     x*sin(angle)+y*cos(angle), 
		                     theta+angle);
    }

	// rotation
    inline RobotLocation operator%=(const double angle) {
        x = x*cos(angle)-y*sin(angle);
        y = x*sin(angle)+y*cos(angle);
		theta += angle;
        return *this;
    }


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

