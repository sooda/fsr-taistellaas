
#include "MapData.hpp"

namespace SLAM {

const int MapData::gridSize = 301;
const double MapData::unitSize = 0.03;

MapData::MapData(RobotLocation initial)
	: xdim(gridSize), ydim(gridSize),
	  xsize(gridSize*unitSize), ysize(gridSize*unitSize),
	  cellxsize(gridSize), cellysize(gridSize),
	  robotLocation(initial),
	  map(std::vector<std::vector<std::vector<double> > >(
				  gridSize, std::vector<std::vector<double> >(
					  gridSize, std::vector<double>(OBS_TYPE_SIZE, -1.0)))
			  )
{
}

// used to set the value of one cell of the map to another
void MapData::setCellValue(GridPoint xy, ObservationType type, double value) {
	if (xy.x > gridSize || xy.x < 0 || xy.y > gridSize || xy.y < 0) { 
		std::cerr << "MapData::setCellValue called with out of bouds value" << std::endl;
		return;
	}
	else {
		map[xy.x][xy.y][type] = value;
	}
}

// used to get the value of one cell of the map
double MapData::getCellValue(GridPoint xy, ObservationType type) const {
	if (xy.x > gridSize || xy.x < 0 || xy.y > gridSize || xy.y < 0) {
		std::cerr << "MapData::getCellValue called with out of bouds value" << std::endl;
		return -1;
	}
	else {
		return map[xy.x][xy.y][type];
	}
}

// used to set the value at some location of the map to another (from nearest cell)
void MapData::setValue(Location xy, ObservationType type, double value) {
	GridPoint xyg = loc2grid(xy);
	setCellValue(xyg, type, value);	
}

// used to get the value at some location of the map (goes to nearest cell)
double MapData::getValue(Location xy, ObservationType type) const {
	GridPoint xyg = loc2grid(xy);
	return getCellValue(xyg, type);
}

// used to set the location of the robot in meters
void MapData::setLocation(RobotLocation xyt) {
	robotLocation = xyt;
}

// used to set the location of the robot
void MapData::setGridLocation(RobotLocation xyt) {
	GridPoint temp (xyt.x, xyt.y);
	Location temp2 = grid2loc(temp);
	setLocation(RobotLocation(temp2.x, temp2.y, xyt.theta));
}

// trnasform a location to a grid point in map
GridPoint MapData::loc2grid(Location xy) {
	int x0 = gridSize/2;
	int y0 = gridSize/2;
	int x = xy.x/unitSize + x0;
	int y = xy.y/unitSize + y0;
	return GridPoint(x,y);
}

// transform a grid point in map to location
Location MapData::grid2loc(GridPoint xy) {
	int x0 = gridSize/2;
	int y0 = gridSize/2;
	double x = (xy.x-x0)*unitSize;
	double y = (xy.y-y0)*unitSize;
	return Location(x,y);
}

// used to get the robot location in meters from center of map
RobotLocation MapData::getRobotLocation() const {
	return robotLocation;
}

// used to get the robot location in the map grid
RobotLocation MapData::getGridLocation() const {
	Location temp (robotLocation.x, robotLocation.y);
	GridPoint temp2 = loc2grid(temp);
	return RobotLocation(temp2.x, temp2.y, robotLocation.theta);
}

}

