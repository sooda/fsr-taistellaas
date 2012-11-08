#include "navi/navigation.hpp"

namespace Navi {

void Navigation::loadMidterm() {
	const float right = 0, up = M_PI/2, left = M_PI, down = 3 * M_PI / 2;
	route.push_back(Pose(3, 4-1.3, right));
	route.push_back(Pose(3.62, 4-2.81, down));
	route.push_back(Pose(3.21, 4-3.41, left));
	route.push_back(Pose(2.72, 4-2.87, up));
	route.push_back(Pose(2.22, 4-2.52, left));
	route.push_back(Pose(1.63, 4-2.91, down));
	route.push_back(Pose(1.01, 4-3.34, left));
	route.push_back(Pose(0.44, 4-2.91, up));
}

Navigation::Navigation() {
	loadMidterm();
}

const Navigation::PoseList& Navigation::getRoute(void) const {
	return route;
}

}

