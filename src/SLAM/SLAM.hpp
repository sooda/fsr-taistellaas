/**

SLAM main interface class

**/

#ifndef _J2B2_SLAM_HPP_
#define _J2B2_SLAM_HPP_

#include "MapData.hpp"
#include "SLAMutil.hpp"
#include <math.h>
#include <signal.h>
#include "J2B2-API.hpp"
#include "gmapping/gfs-j2b2/gfs-j2b2.hpp"
#include <gimutils.h>
#include <../SDL/SDL.h>
#include <../SDL/SDL_image.h>
#include <../SDL/SDL_thread.h>
#include <../SDL/SDL_gfxPrimitives.h>

namespace SLAM {

class SLAM {

public:

// constructor, initializes slam
// takes initial location of the robot as parameter
// map is contructed based on this initial location
SLAM(RobotLocation initial);

// destructor
~SLAM();

// returns the current map data object
MapData getCurrentMapData();

// inform SLAM of the latest laser data
// may also trigger SLAM update
bool updateLaserData(MaCI::Ranging::TDistanceArray laserData);

// inform SLAM of the latest odometry data
void updateOdometryData(RobotLocation delta);

// inform slam of some object at some location
// x and y are in meters in current map coordinates
void updateImageData(ImageData data, MapData::ObservationType type); 

#ifndef _DONT_USE_SDL_
// draws the laser scan on screen
void drawLaserData(SDL_Surface* screen, const int window_width, const int window_height);

// draws map data on the screen
void drawMapData(SDL_Surface* screen, const int window_width, const int window_height);

MaCI::Ranging::TDistance getNearest() const;

#endif

private:

MapData currentMapData;
MaCI::Ranging::TDistanceArray lastLaserData;
RobotLocation lastOdometryData;
MaCI::Ranging::TDistance lastNearest;
gim::time lastOdometryUpdateTime, lastLaserUpdateTime;

int x0, y0, x1, y1;

GMapping::GFSJ2B2 slamThingy;
GMapping::Map<double, GMapping::DoubleArray2D, false>* gfsmap;

};

}

#endif
