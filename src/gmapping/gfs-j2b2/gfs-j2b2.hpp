/*****************************************************************
 *
 * You touch my ding ding dong
 *
 *****************************************************************/


#include "utils/commandline.h"
#include "gridfastslam/gridslamprocessor.h"
#include "utils/orientedboundingbox.h"
#include "configfile/configfile.h"
#include "SLAM/SLAMutil.hpp"
#include "J2B2-API.hpp"

namespace GMapping {

class GFSJ2B2 {

public:

	// constructor
	GFSJ2B2(std::string configfilename);

	// map updating
	void updateMap(MaCI::Ranging::TDistanceArray& array, 
                   SLAM::RobotLocation& loc,
                   Map<double, DoubleArray2D, false>*& newMap,
                   SLAM::RobotLocation& newLoc);

private:

	RangeSensor* frontLaser;
	SensorMap sensorMap;
	GridSlamProcessor* processor;

};


};
