/**

SLAM main interface class

**/

#ifndef _J2B2_SLAM_HPP_
#define _J2B2_SLAM_HPP_

#include "J2B2-API.hpp"
#include "MapData.hpp"
#include "utilities.hpp"
#include <math.h>
#include <signal.h>

#ifndef _DONT_USE_SDL_
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <SDL/SDL_thread.h>
#include <SDL/SDL_gfxPrimitives.h>
#endif


class SLAM {

public:

// constructor, initializes slam
// takes initial location of the robot as parameter
// map is contructed based on this initial location
SLAM(double xsize, double ysize,
     double xdim, double ydim,
     RobotLocation loc,
     MaCI::Ranging::TDistanceArray initial);

// destructor
~SLAM();

// can be called to get the current map data object
MapData getCurrentMapData();

// make slam update map based on laser measurements
void updateLaserData(MaCI::Ranging::TDistanceArray laserData);

// make slam update map based on odometry data
// values should be differences since last calling this function
// (this should only be called by the motion control module)
void updateOdometryData(RobotLocation delta);

// inform slam of some object at some location
// x and y are in meters
void informOfObservation(MapData::ObservationType type, Location xy); 

#ifndef _DONT_USE_SDL_
// draws the laser scan on screen
void drawLaserData(SDL_Surface* screen, const int window_width, const int window_height);
#endif

private:

MapData currentMapData;
MaCI::Ranging::TDistanceArray lastLaserData;
RobotLocation lastOdometryData;


};

#endif
