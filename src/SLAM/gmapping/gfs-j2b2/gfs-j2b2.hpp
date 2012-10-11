/*****************************************************************
 *
 * You touch my ding ding dong
 *
 *****************************************************************/


#include "../utils/commandline.h"
#include "../gridfastslam/gridslamprocessor.h"
#include "../utils/orientedboundingbox.h"
#include "../configfile/configfile.h"
#include "../../SLAMutil.hpp"
#include "../../../J2B2-API.hpp"

namespace GMapping {

class GFSJ2B2 {

public:

	// constructor
	GFSJ2B2(std::string configfilename);

	// map updating
	Map<double, DoubleArray2D, false>* updateMap(MaCI::Ranging::TDistanceArray& array, 
		                                     SLAM::RobotLocation& loc);
private:

	RangeSensor* frontLaser;
	SensorMap sensorMap;
	GridSlamProcessor* processor;

};


};
