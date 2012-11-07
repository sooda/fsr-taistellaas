#ifndef NAVI_NAVIGATION_HPP_
#define NAVI_NAVIGATION_HPP_

#include "SLAM/SLAM.hpp"
#include <list>

namespace Navi {

class Navigation {
public:
	typedef SLAM::RobotLocation Pose;
	typedef std::list<Pose> PoseList;

	Navigation();
	const PoseList& getRoute(void) const;
	// void findRoute(Pose destination); // MIDTERM: not needed yet.

private:
	PoseList route;
	void loadMidterm();

};

}

#endif

