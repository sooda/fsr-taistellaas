
#include "MapData.hpp"

namespace SLAM {

MapData::MapData(int xdim_val, int ydim_val,
                 double xsize_val, double ysize_val,
                 RobotLocation initial)
	: xdim(xdim_val), ydim(ydim_val),
	  xsize(xsize_val), ysize(ysize_val),
	  cellxsize(xsize_val/xdim_val), cellysize(ysize_val/ydim_val),
	  robotLocation(initial) 
{

	for (int i = 0; i < xdim; i++) {
		for (int j = 0; j < ydim; j++) {
			for (int k = 0; k < OBS_TYPE_SIZE; k++) {
				map[i][j][k] = 0.5;
			}
		}
	}

}

// destructor
MapData::~MapData() {

}

// used to set the value of one cell of the map to another
void MapData::setCellValue(GridPoint xy, ObservationType type, double value) {
	map[xy.x][xy.y][type] = value;
}

// used to get the value of one cell of the map
double MapData::getCellValue(GridPoint xy, ObservationType type) {
	return map[xy.x][xy.y][type];
}

// used to set the value at some location of the map to another (from nearest cell)
void MapData::setValue(Location xy, ObservationType type, double value) {
	int x = xy.x/cellxsize;
	int y = xy.y/cellysize;
	map[x][y][type] = value;
}

// used to get the value at some location of the map (goes to nearest cell)
double MapData::getValue(Location xy, ObservationType type) {
	int x = xy.x/cellxsize;
    int y = xy.y/cellysize;
	return map[x][y][type];
}

// used to set the location of the robot
void MapData::setLocation(RobotLocation xyt) {
	robotLocation = xyt;
}

// used to get the robot location
RobotLocation MapData::getLocation() {
	return robotLocation;
}

}

