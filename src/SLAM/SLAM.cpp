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
	// causes crashes sometimes?
	//if (gfsmap) {
	//	delete gfsmap;
	//}
}

// can be called to get the current map data object
const MapData& SLAM::getCurrentMapData() {

	return currentMapData;

}

// make slam update map based on laser measurements
// return true if map was updated
bool SLAM::updateLaserData(MaCI::Ranging::TDistanceArray laserData) {

	if (laserData.empty()) {
		std::cerr << "SLAM: Null laser scan array!" << std::endl;
		return false;
	}
	if (lastOdometryUpdateTime.getSeconds() == 0) {
		std::cerr << "SLAM: No odometry data for update!" << std::endl;
		return false;
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

		//int xmax = gfsmap->getMapSizeX();
		//int ymax = gfsmap->getMapSizeY();
	
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
	
			// fix robot initial position to goal area and free space

			int r_wall = 0.30 / MapData::unitSize; // robot radius in grid points
			int r_wall2 = r_wall*r_wall;
			int r_goal = 0.20 / MapData::unitSize; // approx goal radius
			int r_goal2 = r_goal*r_goal;
			int xInit = MapData::gridSize/2;
			int yInit = MapData::gridSize/2;

			for (int x = -r_wall; x <= r_wall; x++) {
				for (int y = -r_wall; y <= r_wall; y++) {
					int dist2 = x*x+y*y;
					if (dist2 <= r_wall2) {
						currentMapData.setCellValue(GridPoint(xInit+x, yInit+y), MapData::WALL, 0);
					}
					if (dist2 <= r_goal2) {
						currentMapData.setCellValue(GridPoint(xInit+x, yInit+y), MapData::GOAL, 1);
					}
				}
			}
		}

		/*//testing
		ImageData test1(std::vector<Location>(), 
			currentMapData.getRobotLocation(), 0.2, 0.5, 1);
		updateImageData(test1, MapData::TARGET);
  
		ImageData test2(std::vector<Location>(), 
		currentMapData.getRobotLocation(), 0.5, 3, M_PI*66/180);
		updateImageData(test2, MapData::OBSTACLE);

		ImageData test3(std::vector<Location>(), 
			currentMapData.getRobotLocation(), 0.5, 1, 1.5);
		updateImageData(test3, MapData::GOAL);*/
	
		gim::time duration(true);

		duration -= start;

		std::cout << "SLAM: t = " << duration.getSeconds() << "." << duration.getUSeconds() << "s" << std::endl; 
		return true;
	}
	return false;
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
//	double ddist = sqrt(dxy.x*dxy.x+dxy.y*dxy.y);
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

	// preprocessing
	double unusedViewWidth = M_PI*15/180; // 5 degrees from sides off
	double unusedViewDepthFront = MapData::unitSize; // from front off
	double unusedViewDepthBack = MapData::unitSize; // from back off
	data.viewWidth -= unusedViewWidth;
	data.minDist += unusedViewDepthFront;
	data.maxDist -= unusedViewDepthBack;

	double scaleDownDeltas = 0.8;
//	double thetaMin = currentMapData.getRobotLocation().theta - 0.5*data.viewWidth;
//	double thetaMax = currentMapData.getRobotLocation().theta + 0.5*data.viewWidth;
	double thetaMin = data.location.theta - 0.5*data.viewWidth;
	double thetaMax = data.location.theta + 0.5*data.viewWidth;
	double dtheta = scaleDownDeltas*MapData::unitSize/data.maxDist;
	double r0 = 0.2;
	double rMin = data.minDist;
	double rMax = data.maxDist;
	double dr = scaleDownDeltas*MapData::unitSize;

/*	std::cout << "this = " << this << std::endl;
	std::cout << "ImageData, type = " << type << std::endl;
	std::cout << "thetaMin = " << thetaMin << " thetaMax = " << thetaMax << std::endl;
	std::cout << "rmin = " << rMin << " rMax = " << rMax << std::endl;
	std::cout << "dtheta = " << dtheta << " dr = " << dr << std::endl;
	std::cout << "loc = " << data.location << std::endl; */

	double maxObjectDelta = MapData::unitSize*2; // max difference to create a new target
	double maxObjectDelta2 = maxObjectDelta*maxObjectDelta;
	std::vector<Location> objects = std::vector<Location>(currentMapData.getObjects(type));
	std::vector<Location> trash = std::vector<Location>();


	// clear target area
	for (double theta = thetaMin; theta < thetaMax; theta += dtheta) {
		for (double r = r0; r < rMax; r += dr) {

			//double x = currentMapData.getRobotLocation().x + r*cos(theta);
			//double y = currentMapData.getRobotLocation().y + r*sin(theta);
			double x = data.location.x + r*cos(theta);
			double y = data.location.y + r*sin(theta);
			double wall = currentMapData.getValue(Location(x,y), MapData::WALL);
			if (wall > 0.5) {
				currentMapData.setValue(Location(x,y), type, -1.0); // occlusion
				break;
				// move to next theta
			}
			if (wall < -0.5) {
				break; // unknown area
			}
			if (r > rMin) {
				double oldValue = currentMapData.getValue(Location(x,y), type);
				if (oldValue > 0.5) {
					// overwriting a target
					if ((type == MapData::TARGET) || (type == MapData::OBSTACLE)) {
						std::cout << "SLAM: overwriting object at (" << x << ", " << y << ")" << std::endl;
						double hattuvakio = (r/10)*(r/10);
						for (auto it = objects.begin(); it != objects.end(); ++it) {
							double r2 = (x - it->x)*(x - it->x)+(y - it->y)*(y - it->y);
							if (r2 < maxObjectDelta2 + hattuvakio) {
								bool newObjectExists = false;
								for (auto it2 = trash.begin(); it2 != trash.end(); ++it2) {
									double r22 = (x - it2->x)*(x - it2->x)+(y - it2->y)*(y - it2->y);
									if (r22 < maxObjectDelta2 + hattuvakio) {
										newObjectExists = true;
										std::cout << "SLAM: new object (" << type << ": " << it2->x << ", " << it2->y << ") already exists ";
										std::cout << "as (" << type << ": " << it->x << ", " << it->y << "), omitted" << std::endl;
										currentMapData.setValue(Location(x,y), type, 0.0); // empty to avoid double tagging
									}
								}
								for (auto it2 = data.targets.begin(); it2 != data.targets.end();) {
									double r22 = (x - it2->x)*(x - it2->x)+(y - it2->y)*(y - it2->y);
									if (r22 < maxObjectDelta2 + hattuvakio) {
										newObjectExists = true;
										std::cout << "SLAM: new object (" << type << ": " << it2->x << ", " << it2->y << ") already exists ";
										std::cout << "as (" << type << ": " << it->x << ", " << it->y << "), omitted" << std::endl;
										trash.push_back(*it2);
										it2 = data.targets.erase(it2);
										currentMapData.setValue(Location(x,y), type, 0.0); // empty to avoid double tagging
									}
									else {
										++it2;
									}
								}
								if (newObjectExists == false) {
									objects.erase(it);
									std::cout << "SLAM: object (" << type << ": " << it->x << ", " << it->y << ") removed" << std::endl;
									currentMapData.setValue(Location(x,y), type, 0.0); // empty
								}
								break;
							}
						}
					}
					if (type == MapData::GOAL) {
						// goal points are redrawn later
						currentMapData.setValue(Location(x,y), type, 0.0);
					}
				}
				else {
					currentMapData.setValue(Location(x,y), type, 0.0); // empty
				}
			}
		}
	}

	// discard improbable targets
	for (auto it = data.targets.begin(); it != data.targets.end();) {
		double wallValue = currentMapData.getValue(Location(it->x,it->y), MapData::WALL);
		if (fabs(wallValue) > 0.5) { // wall or unknown
			std::cout << "SLAM: new object (" << type << ": " << it->x << ", " << it->y << ") in unprobable location, omitted" << std::endl;
			it = data.targets.erase(it);
		}
		else {
			++it;
		}
	}

	// discard targets under robot
	double robotRad2 = 0.24*0.24;
	for (auto it = objects.begin(); it != objects.end();) {
		double x = currentMapData.getRobotLocation().x;
		double y = currentMapData.getRobotLocation().y;
		double r2 = (x - it->x)*(x - it->x)+(y - it->y)*(y - it->y);
		if (r2 < robotRad2) { // under robot
			std::cout << "SLAM: object (" << type << ": " << it->x << ", " << it->y << ") under robot, removed" << std::endl;
			currentMapData.setValue(Location(it->x,it->y), type, 0.0); // set empty
			it = objects.erase(it);
		} 
		else {
			++it;
		}
	}

	// integrate new target data with old
	if ((type == MapData::TARGET) || (type == MapData::OBSTACLE)) {
		for (auto it = data.targets.begin(); it != data.targets.end(); ++it) {
			double x = it->x;
			double y = it->y;

			bool duplicate = false;
			for (auto it2 = objects.begin(); it2 != objects.end(); ++it2) {
				double r2 = (x - currentMapData.getRobotLocation().x)*(x - currentMapData.getRobotLocation().x)+(y - currentMapData.getRobotLocation().y)*(y - currentMapData.getRobotLocation().y);
				double hattuvakio = r2/100;
				double objectDelta2 = (x - it2->x)*(x - it2->x)+(y - it2->y)*(y - it2->y);
	 			if (objectDelta2 < maxObjectDelta2 + hattuvakio) {
					duplicate = true;
					std::cout << "SLAM: new object (" << type << ": " << x << ", " << y << ") is duplicate with";
					std::cout << " (" << type << ": " << it2->x << ", " << it2->y << "), omitted" << std::endl;
					break;
				}
			}
	
			if (duplicate == false) {
				std::cout << "SLAM: new object (" << type << ": " << x << ", " << y << ") added" << std::endl;
				objects.push_back(Location(x,y));
			}	
		}
	
	}
	if ((type == MapData::GOAL) && (data.targets.size() > 0)) {
		double width2 = 0.48*0.48;
		double height2 = 0.40*0.40;

		if (data.targets.size() == 4) {
			Location c1 = data.targets[0];
			Location c2 = data.targets[1];
			Location c3 = data.targets[2];
			Location c4 = data.targets[3];
			Location swap = c1;

			double d12 = (c1.x-c2.x)*(c1.x-c2.x)+(c1.y-c2.y)*(c1.y-c2.y);
			double d13 = (c1.x-c3.x)*(c1.x-c3.x)+(c1.y-c3.y)*(c1.y-c3.y);
			double d14 = (c1.x-c4.x)*(c1.x-c4.x)+(c1.y-c4.y)*(c1.y-c4.y);
			double swapd;

			// make c3 be the diagonal point from c1 and c1-c2 the shorter edge
			if (d12 > d13) {
				swap = c3;
				swapd = d13;
				c3 = c2;
				d13 = d12;
				c2 = swap;
				d12 = swapd;
			}
			if (d14 > d13) {
				swap = c3;
				swapd = d13;
				c3 = c4;
				d13 = d14;
				c4 = swap;
				d14 = swapd;
			}
			if (d12 > d14) {
				swap = c4;
				swapd = d14;
				c4 = c2;
				d14 = d12;
				c2 = swap;
				d12 = swapd;
			}

			std::cout << "SLAM: goal at c1: (" << c1.x << ", " << c1.y << ")";
			std::cout << ", c2: (" << c2.x << ", " << c2.y << ")";
			std::cout << ", c3: (" << c3.x << ", " << c3.y << ")";
			std::cout << ", c4: (" << c4.x << ", " << c4.y << ")" << std::endl;

			std::cout << "SLAM: height: " << sqrt(d12) << " and " << sqrt(d13-d14) << ", width: " << sqrt(d14) << " and " << sqrt(d13-d12) << std::endl;

			// check rationality of points
			// TODO fix the thing that causes edge lengths to be wrong
			bool rational = true;
			if (fabs(d12-height2) > 0.04) {
				// fucking fabsulous
				std::cout << "SLAM: goal short edge c1-c2 wrong size (" << sqrt(d12) << ")" << std::endl;
				rational = false;
			}
			if (fabs(d13-d14-height2) > 0.04) { // pythagoras, assuming ~90 deg angles
				std::cout << "SLAM: goal short edge c3-c4 wrong size(" << sqrt(d13-d14) << ")" << std::endl;
				rational = false;
			}
			if (fabs(d14-width2) > 0.04) {
				std::cout << "SLAM: goal long edge c1-c4 wrong size(" << sqrt(d14) << ")" << std::endl;
				rational = false;
			}
			if (fabs(d13-d12-width2) > 0.04) { // pythagoras, assuming ~90 deg angles
				std::cout << "SLAM: goal long edge c2-c3 wrong size(" << sqrt(d13-d12) << ")" << std::endl;
				rational = false;
			}

			if (rational == false) {
				std::cout << "SLAM: omitting this goal data" << std::endl;
			}
			else {
				Location center = Location((c1.x+c2.x+c3.x+c4.x)/4, (c1.y+c2.y+c3.y+c4.y)/4);
				std::cout << "SLAM: goal center now at (" << center.x << ", " << center.y << ")" << std::endl;

				objects.clear();
				objects.push_back(center);
				objects.push_back(c1);
				objects.push_back(c2);
				objects.push_back(c3);
				objects.push_back(c4);
			}
		} 
		else {
			std::cout << "SLAM: invalid amount of goal corner points given (" << data.targets.size() << ")" << std::endl;
		}
	}

	// redraw targets
	if ((type == MapData::TARGET) || (type == MapData::OBSTACLE)) {
		for (auto it = objects.begin(); it != objects.end(); ++it) {
			currentMapData.setValue(Location(it->x, it->y), type, 1.0); // target
		}
	}
	if (type == MapData::GOAL) {
		if (objects.size() == 5) {
			Location c1 = objects[1];
			Location c2 = objects[2]; // we don't need c3, actually
			Location c4 = objects[4];

			for (double i = 0; i <= 1; i+=MapData::unitSize*0.9) {
				for (double j = 0; j <= 1; j+=MapData::unitSize*0.9) {
					double x = c1.x + (c2.x-c1.x)*i + (c4.x-c1.x)*j;
					double y = c1.y + (c2.y-c1.y)*i + (c4.y-c1.y)*j;
					currentMapData.setValue(Location(x,y), type, 1.0);
				}
			}
		}
	}

	// fix robot initial position to goal area and free space

	int r_wall = 0.30 / MapData::unitSize; // robot radius in grid points
	int r_wall2 = r_wall*r_wall;
	int r_goal = 0.20 / MapData::unitSize; // approx goal radius
	int r_goal2 = r_goal*r_goal;
	int xInit = MapData::gridSize/2;
	int yInit = MapData::gridSize/2;

	for (int x = -r_wall; x <= r_wall; x++) {
		for (int y = -r_wall; y <= r_wall; y++) {
			int dist2 = x*x+y*y;
			if (dist2 <= r_wall2) {
				currentMapData.setCellValue(GridPoint(xInit+x, yInit+y), MapData::WALL, 0);
			}
			if (dist2 <= r_goal2) {
				currentMapData.setCellValue(GridPoint(xInit+x, yInit+y), MapData::GOAL, 1);
			}
		}
	}


	// update targets to map data
	currentMapData.setObjects(objects, type);
}


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

	const int gridsz = MapData::gridSize;
	for (int type_ = MapData::WALL; type_ != MapData::OBS_TYPE_SIZE; type_++) {
		MapData::ObservationType type = (MapData::ObservationType)type_;

		int x0 = 0;
		int y0 = 0;

		if (type == MapData::TARGET) {
			x0 = gridsz + 10;
		}
		if (type == MapData::OBSTACLE) {
			y0 = gridsz + 10;
		}
		if (type == MapData::GOAL) {
			x0 = gridsz + 10;
			y0 = gridsz + 10;
		}

		// draw the map
		lineRGBA(screen, x0, y0, x0 + gridsz + 1, y0, 255, 255, 255, 255);
		lineRGBA(screen, x0, y0 + gridsz + 1, x0 + gridsz + 1, y0 + gridsz + 1, 255, 255, 255, 255);
		lineRGBA(screen, x0, y0, x0, y0 + gridsz + 1, 255, 255, 255, 255);
		lineRGBA(screen, x0 + gridsz + 1, y0, x0 + gridsz + 1, y0 + gridsz + 1, 255, 255, 255, 255);
		x0++; y0++;
		for (int x = 0; x < gridsz; x++) {
			for (int y = 0; y < gridsz; y++) {
				double wall = currentMapData.getCellValue(GridPoint(x,y), type);
				int colorR = (int)(150+wall*100);
				int colorG = (int)(150+wall*100);
				int colorB = (int)(150+wall*100);

				if (type == MapData::TARGET) colorR = 0;
				if (type == MapData::OBSTACLE) colorG = 0;
				if (type == MapData::GOAL) colorB = 0;

				pixelRGBA(screen,
					x0 + x,
					y0 + gridsz-1 - y,
					colorR, colorG, colorB, 255);
			}
		}
	
		// draw the robot
		RobotLocation loc = currentMapData.getGridLocation();
		filledCircleRGBA(screen, 
			x0 + loc.x, 
			y0 + gridsz-1 - loc.y, 
			(int)(0.4 / MapData::unitSize / 2), 0, 255, 255, 255);

		// draw robot direction line
	    for (int i = 0; i < 10; i++) { 
			pixelRGBA(screen, 
				x0 + loc.x+i*cos(loc.theta), 
				y0 + gridsz-1 - loc.y-i*sin(loc.theta),
				255, 0, 0, 255);
		}  

	}
   
}

MaCI::Ranging::TDistance SLAM::getNearest() const {
	return lastNearest;
}

}

