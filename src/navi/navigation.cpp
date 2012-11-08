#include "navi/navigation.hpp"
#include "util.hpp"

namespace Navi {

void Navigation::loadMidterm() {
	route.push_back(Pose(3.00, 2.70, DIR_RIGHT));
	route.push_back(Pose(3.62, 1.19, DIR_DOWN));
	route.push_back(Pose(3.21, 0.59, DIR_LEFT));
	route.push_back(Pose(2.72, 1.13, DIR_UP));
	route.push_back(Pose(2.22, 1.48, DIR_LEFT));
	route.push_back(Pose(1.63, 1.06, DIR_DOWN));
	route.push_back(Pose(1.01, 0.66, DIR_LEFT));
	route.push_back(Pose(0.44, 1.09, DIR_UP));
}

Navigation::Navigation() {
	loadMidterm();
}

const Navigation::PoseList& Navigation::getRoute(void) const {
	return route;
}

}

