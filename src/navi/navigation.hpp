#ifndef NAVI_NAVIGATION_HPP_
#define NAVI_NAVIGATION_HPP_

#include "SLAM/includes.hpp"
#include <list>
#include "navi/util/dilate.hpp"
#include "navi/util/gridutil.hpp"
#include "navi/util/gridsearch.hpp"

namespace Navi {

class Navigation {
public:
	typedef std::list<SLAM::Location> LocList;

	Navigation();
	void refreshMap(const SLAM::MapData& data);
	void draw(SDL_Surface* surface, int posx, int posy) const;
	bool solveTo(SLAM::Location loc);
	bool solveTo(SLAM::GridPoint point);
	float routeLength(SLAM::Location loc);
	LocList getRoute(void) const;
	void updateLocation(SLAM::RobotLocation pos);
	bool isFloor(SLAM::Location loc) const;
	float wallClearance(SLAM::RobotLocation source) const;
private:
	search_info findRoute(SLAM::GridPoint point);
	SLAM::MapData map;
	GridMap wallmap_orig;
	GridMap wallmap_dila;
	std::list<gridvertex> current_route;
	SLAM::RobotLocation roboPos;
};

}

#endif

