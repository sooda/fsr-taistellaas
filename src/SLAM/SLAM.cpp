/**

SLAM demo by Antti

**/

#include "SLAM.hpp"

namespace SLAM {

// constructor, initializes slam
// takes initial location of the robot as parameter
// map is contructed based on this initial location
SLAM::SLAM(double xsize, double ysize, 
           double xdim, double ydim, 
           RobotLocation loc,
           MaCI::Ranging::TDistanceArray initial) 
	: currentMapData(xdim, ydim, xsize, ysize, loc),
	  lastLaserData(initial),
	  lastOdometryData(RobotLocation(0,0,0)),
	  lastNearest(),
	  slamThingy(std::string())
{

   
    
}

// can be called to get the current map data object
MapData SLAM::getCurrentMapData() {

	return currentMapData;

}

// make slam update map based on laser measurements
void SLAM::updateLaserData(MaCI::Ranging::TDistanceArray laserData) {

	lastLaserData = laserData;

 	GMapping::Map<double, GMapping::DoubleArray2D, false>* gfsmap;

	//gfsmap = slamThingy.updateMap(lastLaserData, lastOdometryData);

	if (gfsmap != 0) {
		// map was updated

		int xsize = gfsmap->getMapSizeX();
		int ysize = gfsmap->getMapSizeY();

		for (int x = 0; x < xsize; x++) {
			for (int y = 0; y < ysize; y++) {
				currentMapData.setCellValue(GridPoint(x,y), MapData::WALL, gfsmap->cell(x,y));
			}
		}
	}	

}

// make slam update map based on odometry data
// (this should only be called by the motion control module)
void SLAM::updateOdometryData(RobotLocation loc) {

	lastOdometryData = loc;

	//slamThingy.updateMap(lastLaserData, lastOdometryData);

}

// inform slam of some object at some location
// x and y are in meters
void SLAM::informOfObservation(MapData::ObservationType type, Location xy) {

	// magic happens here

}


#ifndef _DONT_USE_SDL_
// draws the laser scan on screen
void SLAM::drawLaserData(SDL_Surface* screen, const int window_width, const int window_height) {
  
	if (lastLaserData.size()) {
		float min_d = 1000;
		MaCI::Ranging::TDistance min_dist;
		float scale = 160; // scales from meters to screen pixels
		int min_x_end = 0;
		int min_y_end = 0;
		int x_origin = window_width/2;
		int y_origin = window_height/2;
		int dasize = lastLaserData.size();
		for(int i = 0; i < dasize; ++i) {
			const MaCI::Ranging::TDistance &measurement = lastLaserData[i];
			float dscale = 1;
			for (float cscale = 1; cscale < scale; cscale += dscale) {
				int pix_x = x_origin - (int)(cscale * measurement.distance * sin(measurement.angle)); 
				int pix_y = y_origin - (int)(cscale * measurement.distance * cos(measurement.angle)); 
				if (pix_x >= 0 && pix_x < window_width && pix_y >= 0 && pix_y < window_height) {
					// draw the empty space with white dots
					pixelRGBA(screen, pix_x, pix_y, 255, 255, 255, 150);
				}
			}
			int pix_x = x_origin - (int)(scale * measurement.distance * sin(measurement.angle));
			int pix_y = y_origin - (int)(scale * measurement.distance * cos(measurement.angle));	
			if (pix_x >= 0 && pix_x < window_width && pix_y >= 0 && pix_y < window_height) {
				// draw the wall pixel with red dot and circle
				pixelRGBA(screen, pix_x, pix_y, 255, 0, 0, 150);
				filledCircleRGBA(screen, pix_x, pix_y, 5, 255, 0, 0, 150);
				if (measurement.distance < min_d) {
					min_d = measurement.distance;
					min_x_end = pix_x;
					min_y_end = pix_y;
					min_dist = measurement;
				}
			}
		}
		
		// draw the robot
		filledCircleRGBA(screen, x_origin, y_origin, (int)(0.2 * scale), 0, 255, 255, 255);
      
		// draw distance to nearest object
		lineRGBA(screen,
		 x_origin, y_origin-1,
		 min_x_end, min_y_end,
		 255, 255, 0, 255);

		lastNearest = min_dist;
	}
    
}

// draws the map on screen
void SLAM::drawMapData(SDL_Surface* screen, const int window_width, const int window_height) {
 
	for (int x = 0; x < 100; x++) {
		for (int y = 0; y < 100; y++) {
			double wall = currentMapData.getCellValue(GridPoint(x,y), MapData::WALL);
			int color = (int)(wall*255);
			pixelRGBA(screen, x, y, color, color, 255, 150);
		}
	}
   
}
#endif

MaCI::Ranging::TDistance SLAM::getNearest() const {
	return lastNearest;
}

}

