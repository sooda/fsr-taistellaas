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
  currentMapData.setGridLocation(RobotLocation(MapData::gridSize/2,MapData::gridSize/2,0));
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

			x0 = newLoc.x - MapData::gridSize/2;
			x1 = newLoc.x + MapData::gridSize/2;
			y0 = newLoc.y - MapData::gridSize/2;
			y1 = newLoc.y + MapData::gridSize/2;

		}
	
		if(x0 != -1 && y0 != -1 && x1 != -1 && y1 != -1) {	
	
			newLoc.x -= x0;
			newLoc.y -= y0;
			
			currentMapData.setGridLocation(newLoc);
			
			for (int x = x0; x <= x1; x++) {
				for (int y = y0; y <= y1; y++) {
					currentMapData.setCellValue(GridPoint(x-x0,y-y0), MapData::WALL, gfsmap->cell(x,y));				
				}
			}
		}

		// testing
		ImageData test1(std::vector<std::pair<double,double> >(), 
			currentMapData.getRobotLocation(), 1, 2, 1);
		updateImageData(test1, MapData::TARGET);
  
		ImageData test2(std::vector<std::pair<double,double> >(), 
			currentMapData.getRobotLocation(), 2, 4, 0.5);
		updateImageData(test2, MapData::OBSTACLE);

		ImageData test3(std::vector<std::pair<double,double> >(), 
			currentMapData.getRobotLocation(), 0.5, 1, 1.5);
		updateImageData(test3, MapData::GOAL);
	
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

/*  std::cout << "---" << std::endl;
	RobotLocation test (1,0,0);
	std::cout << "test: " << test << std::endl;	
	test %= M_PI/2;
	std::cout << "test: " << test << std::endl;	
	test %= M_PI/2;
	std::cout << "test: " << test << std::endl;	
	test %= M_PI/2;
	std::cout << "test: " << test << std::endl;	*/

//	std::cout << "---" << std::endl;	
//	std::cout << "ddist: " << ddist << ", dtheta: " << dtheta << std::endl;
//	std::cout << "dxy: " << dxy << std::endl; 
	dxy %= -lastLoc.theta;
//	std::cout << "dxyr: " << dxy << std::endl; 
//	std::cout << "loc: " << loc << std::endl; 
	
	lastLoc = loc;

	RobotLocation dxyo = dxy % lastOdometryData.theta;
	dxyo.theta = dtheta;
//	std::cout << "dxyo " << dxy.x << " " << dxy.y << " " << dxy.theta << std::endl;
	lastOdometryData += dxyo;
	lastOdometryData.normalizeTheta();

	RobotLocation maploc = currentMapData.getRobotLocation();
	RobotLocation dxym = dxy % maploc.theta;
	dxym.theta = dtheta;
//	std::cout << "dxyo " << dxy.x << " " << dxy.y << " " << dxy.theta << std::endl;
	maploc += dxym;
	maploc.normalizeTheta();
	currentMapData.setLocation(maploc);

//	std::cout << lastLoc.x << "," << lastLoc.y << "," << lastLoc.theta << " " << 
//		lastOdometryData.x << "," << lastOdometryData.y << "," << lastOdometryData.theta << std::endl;

}

// inform slam of some object at some location
void SLAM::updateImageData(ImageData data, MapData::ObservationType type) {

	double scaleDownDeltas = 0.9;
	double thetaMin = data.location.theta - 0.5*data.viewWidth;
	double thetaMax = data.location.theta + 0.5*data.viewWidth;
	double dtheta = scaleDownDeltas*MapData::unitSize/(data.viewWidth*data.maxDist);
	double rMin = data.minDist;
	double rMax = data.maxDist;	
	double dr = scaleDownDeltas*MapData::unitSize;

	std::cout << "ImageData " << type << std::endl;
	std::cout << "thetaMin = " << thetaMin << " thetaMax = " << thetaMax << std::endl;
	std::cout << "rmin = " << rMin << " rMax = " << rMax << std::endl;
	std::cout << "dtheta = " << dtheta << " dr = " << dr << std::endl;
	std::cout << "loc = " << data.location << std::endl;

	for (double theta = thetaMin; theta < thetaMax; theta += dtheta) {
		for (double r = rMin; r < rMax; r += dr) {
			double x = data.location.x + r*cos(theta);
			double y = data.location.y + r*sin(theta);
	//		std::cout << "x = " << x << " y = " << y << std::endl;
			double wall = currentMapData.getValue(Location(x,y), MapData::WALL);
			if (wall > 0.5) {
				currentMapData.setValue(Location(x,y), type, -1.0); // occlusion
				break;
				// move to next theta
			}
			if (wall < -0.5) {
				break; // unknown area
			}
			currentMapData.setValue(Location(x,y), type, 0.0); // empty
		}
	}

	for (auto it = data.targets.begin(); it != data.targets.end(); ++it) {
		double x = data.location.x + it->first;
		double y = data.location.y + it->second;
		for (int i = -1; i < 2; i++) {
			for (int j = -1; j < 2; j++) {
				currentMapData.setValue(Location(x+i,y+j), type, 1.0); // target
			}
		}
	}

}


#ifndef _DONT_USE_SDL_
// draws the laser scan on screen
void SLAM::drawLaserData(SDL_Surface* screen, const int window_width, const int window_height) {
  
	if (lastLaserData.size()) {
		float min_d = 1000;
		MaCI::Ranging::TDistance min_dist;
		float scale = 50; // scales from meters to screen pixels
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
				filledCircleRGBA(screen, pix_x, pix_y, 3, 255, 0, 0, 150);
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

	for (int type = 0; type < 4; type++) {

		int x0 = 0;
		int y0 = MapData::gridSize;

		if (type == 1) {
			x0 = window_width - MapData::gridSize;
			y0 = MapData::gridSize;
		}
		if (type == 2) {
			x0 = 0;
			y0 = window_height;
		}
		if (type == 3) {
			x0 = window_width - MapData::gridSize;
			y0 = window_height;
		}

		// draw the map
		for (int x = 0; x < MapData::gridSize; x++) {
			for (int y = 0; y < MapData::gridSize; y++) {
				double wall = currentMapData.getCellValue(GridPoint(x,y), 
					(MapData::ObservationType)type);
				int colorR = (int)(150+wall*100);
				int colorG = (int)(150+wall*100);
				int colorB = (int)(150+wall*100);

				if (type == 1) colorR = 0;
				if (type == 2) colorG = 0;
				if (type == 3) colorB = 0;

				pixelRGBA(screen, 
					x0 + x, 
					y0 - y, 
					colorR, colorG, colorB, 100);	
			}
		}
	
		// draw the robot
		RobotLocation loc = currentMapData.getGridLocation();
		filledCircleRGBA(screen, 
			x0 + loc.x, 
			y0 - loc.y, 
			(int)(0.4 / MapData::unitSize / 2), 0, 255, 255, 255);

		// draw robot direction line
	    for (int i = 0; i < 10; i++) { 
			pixelRGBA(screen, 
				x0 + loc.x+i*cos(loc.theta), 
				y0 - loc.y-i*sin(loc.theta),
				255, 0, 0, 255);
		}  

	}
   
}
#endif

MaCI::Ranging::TDistance SLAM::getNearest() const {
	return lastNearest;
}

}

