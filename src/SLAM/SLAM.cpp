/**

SLAM demo by Antti

**/

#include "SLAM.hpp"

namespace SLAM {

// constructor, initializes slam
// takes initial location of the robot as parameter
// map is contructed based on this initial location
SLAM::SLAM(RobotLocation initial) 
	: currentMapData(initial),
	  lastLaserData(),
	  lastOdometryData(RobotLocation(0,0,0)),
	  lastNearest(),
	  lastOdometryUpdateTime(0), 
	  lastLaserUpdateTime(0),
	  x0(-1), 
	  y0(-1),
	  x1(-1),
	  y1(-1),
	  slamThingy("gmapping/ini/gfs-LMS-j2b2.ini"), // TODO: don't hardcode this here
	  gfsmap(NULL)
{
  currentMapData.setLocation(RobotLocation(MapData::gridSize/2,MapData::gridSize/2,0));
}

// destructor
SLAM::~SLAM() {
	if (gfsmap) {
		delete gfsmap;
	}
}

// can be called to get the current map data object
MapData SLAM::getCurrentMapData() {

	return currentMapData;

}

// make slam update map based on laser measurements
void SLAM::updateLaserData(MaCI::Ranging::TDistanceArray laserData) {

	if (laserData.empty()) {
		std::cerr << "SLAM: Null laser scan array!" << std::endl;
		return;
	}
	if (lastOdometryUpdateTime.getSeconds() == 0) {
		std::cerr << "SLAM: No odometry data for update!" << std::endl;
		return;
	}


	gim::time start(true);

	lastLaserUpdateTime.setToCurrent();

	lastLaserData = laserData;

	GMapping::Map<double, GMapping::DoubleArray2D, false>* new_gfsmap = 0;	

	RobotLocation newLoc = RobotLocation(0,0,0);

	slamThingy.updateMap(lastLaserData, lastOdometryData, new_gfsmap, newLoc);
	
	if (new_gfsmap != 0) {
		// map was updated

		std::cerr << "SLAM: map updated." << std::endl;

		delete gfsmap;
		gfsmap = new_gfsmap;

		int xmax = gfsmap->getMapSizeX();
		int ymax = gfsmap->getMapSizeY();
	
		if (x0 == -1 && y0 == -1 && x1 == -1 && y1 == -1) {
			x0 = 0;
			y0 = 0;
			x1 = xmax;
			y1 = ymax;
	
			for (int x = 0; x < xmax; x++) {
				for (int y = 0; y < ymax; y++) {
					if (gfsmap->cell(x,y) > 0.5) {
						x0 = x;
						goto end1;
					}
				}
			}
			end1:
	
			for (int x = xmax-1; x >= 0; x--) {
				for (int y = 0; y < ymax; y++) {
					if (gfsmap->cell(x,y) > 0.5) {
						x1 = x;
						goto end2;
					}
				}
			}
			end2:
			
			for (int y = 0; y < ymax; y++) {
				for (int x = 0; x < xmax; x++) {
					if (gfsmap->cell(x,y) > 0.5) {
						y0 = y;
						goto end3;
					}
				}
			} 
			end3:
	
			for (int y = ymax-1; y >= 0; y--) {
				for (int x = 0; x < xmax; x++) {
					if (gfsmap->cell(x,y) > 0.5) {
						y1 = y;
						goto end4;
					}
				}
			}
			end4:
	
			while (x1-x0 < MapData::gridSize-2) {
				if (x0 > 0) {
					x0--;
				}
				if (x1 < xmax) {
					x1++;
				}
			}	

			while (y1-y0 < MapData::gridSize-2) {
				if (y0 > 0) {
					y0--;
				}
				if (y1 < ymax) {
					y1++;
				}
			}
		}
	
		if(x0 != -1 && y0 != -1 && x1 != -1 && y1 != -1) {	
	
			newLoc.x -= x0;
			newLoc.y -= y0;
			
			currentMapData.setLocation(newLoc);
			
			for (int x = x0; x <= x1; x++) {
				for (int y = y0; y <= y1; y++) {
					currentMapData.setCellValue(GridPoint(x-x0,y-y0), MapData::WALL, gfsmap->cell(x,y));				
				}
			}
		}

	
		gim::time duration(true);

		duration -= start;

		std::cout << "SLAM: t = " << duration.getSeconds() << "." << duration.getUSeconds() << "s" << std::endl; 

	}	
	else {
		//std::cerr << "SLAM: no new map" << std::endl;
	}

}

// make slam update map based on odometry data
void SLAM::updateOdometryData(RobotLocation loc) {

	static RobotLocation lastLoc;
	loc.normalizeTheta();
	
	if (lastOdometryUpdateTime == gim::time(0)) {
		lastLoc = loc;
	}

	lastOdometryUpdateTime.setToCurrent();
	
	RobotLocation dxy = loc - lastLoc;	
	dxy.normalizeTheta();
	double ddist = sqrt(dxy.x*dxy.x+dxy.y*dxy.y);
	double dtheta = dxy.theta;
	lastLoc = loc;

	double avgAngleOdo = lastOdometryData.theta + 0.5 * dtheta;
	RobotLocation odoChange(ddist*cos(avgAngleOdo), ddist*sin(avgAngleOdo), 0);
	lastOdometryData += odoChange;
	lastOdometryData.theta += dtheta;
	lastOdometryData.normalizeTheta();

	RobotLocation gridLoc = currentMapData.getLocation();
	double avgAngleMap = gridLoc.theta + 0.5 * dtheta;
	RobotLocation mapChange(ddist*cos(avgAngleMap), ddist*sin(avgAngleMap), 0);
	mapChange /= MapData::unitSize;
	gridLoc += mapChange;
	gridLoc.theta += dtheta;
	gridLoc.normalizeTheta();
	currentMapData.setLocation(gridLoc);

//	std::cout << lastLoc.x << "," << lastLoc.y << "," << lastLoc.theta << " " << 
//		lastOdometryData.x << "," << lastOdometryData.y << "," << lastOdometryData.theta << std::endl;

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

	if (gfsmap == 0) {
		return;
	}

	// draw the map
	for (int x = 0; x < MapData::gridSize; x++) {
		for (int y = 0; y < MapData::gridSize; y++) {
			double wall = currentMapData.getCellValue(GridPoint(x,y), MapData::WALL);
			int color = (int)(150+wall*100);
			pixelRGBA(screen, x, MapData::gridSize-y, color, color, color, 100);	
		}
	}

	// draw the robot
	RobotLocation loc = currentMapData.getLocation();
	filledCircleRGBA(screen, loc.x, MapData::gridSize-loc.y, (int)(5), 0, 255, 255, 255);
    for (int i = 0; i < 10; i++) {
		pixelRGBA(screen, loc.x+i*cos(loc.theta), 
		                  MapData::gridSize-loc.y-i*sin(loc.theta),
		                  255, 0, 0, 255);
	}  

   
}
#endif

MaCI::Ranging::TDistance SLAM::getNearest() const {
	return lastNearest;
}

}

