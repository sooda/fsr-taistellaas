/*

Map Data Object

Antti 26.9.

*/

#ifndef _J2B2_MAPDATA_HPP_
#define _J2B2_MAPDATA_HPP_

#include <iostream>
#include "SLAMutil.hpp"

namespace SLAM {

class MapData {

public:

// type of observation
enum ObservationType {
        WALL=0,
        TARGET=1,
        OBSTACLE=2,
        GOAL=3,
        OBS_TYPE_SIZE=4  // the size of this enumeration
};

// constructor
MapData(RobotLocation initial);

// used to set the value of one cell of the map to another
void setCellValue(GridPoint xy, ObservationType type, double value);

// used to get the value of one cell of the map
double getCellValue(GridPoint xy, ObservationType type);

// used to set the value at some location of the map to another (from nearest cell)
void setValue(Location xy, ObservationType type, double value);

// used to get the value at some location of the map (goes to nearest cell)
double getValue(Location xy, ObservationType type);

// trnasform a location to a grid point in map
static GridPoint loc2grid(Location xy);

// transform a grid point in map to location
static Location grid2loc(GridPoint xy);

// used to set the location of the robot in meters
void setLocation(RobotLocation xyt);

// used to set the location of the robot in the grid
void setGridLocation(RobotLocation xyt);

// used to get the robot location in meters from center of map
RobotLocation getRobotLocation();
	
// used to get the robot location in the map grid
RobotLocation getGridLocation();
	

constexpr static int gridSize = 301;
constexpr static double unitSize = 0.03;

private:

int xdim, ydim;			// dimension of the map (cells)
double xsize, ysize;		// actual size of the map (meters)
double cellxsize, cellysize;	// size of one cell (meters)

RobotLocation robotLocation;	// robot location in the map

double map [gridSize][gridSize][OBS_TYPE_SIZE];  // map data

};

}

#endif


