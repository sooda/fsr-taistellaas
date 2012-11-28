/*

Utility stuff and such

*/

#ifndef _J2B2_UTILITIES_HPP_
#define _J2B2_UTILITIES_HPP_

#include <cmath>
#include <utility>

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
		double x_ = x;
		double y_ = y;
        x = x_*cos(angle)-y_*sin(angle);
        y = x_*sin(angle)+y_*cos(angle);
		theta += angle;
        return *this;
    }

};

inline std::ostream& operator<<(std::ostream& os, const RobotLocation& obj) {
	os << "(" << obj.x << "," << obj.y << "," << obj.theta << ")";
	return os;
}

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

class ImageData {

public:

	ImageData(std::vector<std::pair<double, double> > targets_,
		RobotLocation location_, double minDist_, 
		double maxDist_, double viewWidth_)
		: targets(targets_), location(location_), minDist(minDist_),
		  maxDist(maxDist_), viewWidth(viewWidth_) { }

	std::vector<std::pair<double, double> > targets; 
	// identified targets relative to the place where the image was taken
	RobotLocation location; // where the image was taken
	double minDist; // distance to the earer edge of the image in m
	double maxDist; // distance to the farther edge of the image in m
	double viewWidth; // width of the image in rad

};

}

#endif

