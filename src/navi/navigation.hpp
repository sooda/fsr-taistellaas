#ifndef NAVI_NAVIGATION_HPP_
#define NAVI_NAVIGATION_HPP_

#include "SLAM/includes.hpp"
#include <list>
#include "navi/util/dilate.hpp"
#include "navi/util/gridutil.hpp"

namespace Navi {

class Navigation {
public:
	typedef SLAM::RobotLocation Pose;
	typedef std::list<Pose> PoseList;

	Navigation();
	void refreshMap(const SLAM::MapData& data);
	void draw(SDL_Surface* surface, int posx, int posy) const;
	void solveGrid(int x, int y);
	const PoseList& getRoute(void) const;
	// void findRoute(Pose destination); // MIDTERM: not needed yet.

private:
	PoseList route;
	void loadMidterm();
	SLAM::MapData map;
	GridMap wallmap_orig;
	GridMap wallmap_dila;
	std::list<gridvertex> current_route;
};

}

#endif

