/*****************************************************************
 *
 * Uh ah tralala
 *
 * This if the wrapper for the rao-blackwellian particle filter
 * a.k.a. gmapping
 *
 *****************************************************************/

#include "gfs-j2b2.hpp"

namespace GMapping {

using namespace std;

GFSJ2B2::GFSJ2B2(std::string configfilename) {
	
	std::string outfilename="";
	double xmin=-100.;
	double ymin=-100.;
	double xmax=100.;
	double ymax=100.;
	double delta=0.05;
	
	//scan matching parameters
	double sigma=0.05;
	double maxrange=80.;
	double maxUrange=80.;
	double regscore=1e4;
	double lstep=.05;
	double astep=.05;
	int kernelSize=1;
	int iterations=5;
	double critscore=0.;
	double maxMove=1.;
	double lsigma=.075;
	double ogain=3;
	int lskip=0;

	//motion model parameters
	double srr=0.01, srt=0.01, str=0.01, stt=0.01;
	//particle parameters
	int particles=30;
	
	
	//gfs parameters
	double angularUpdate=0.5;
	double linearUpdate=1;
	double resampleThreshold=0.5;
	bool generateMap=true;

	if (configfilename.length()>0){
	  ConfigFile cfg(configfilename);
	  outfilename = (std::string) cfg.value("gfs","outfilename",outfilename);
	  xmin = cfg.value("gfs","xmin", xmin);
	  xmax = cfg.value("gfs","xmax",xmax);
	  ymin = cfg.value("gfs","ymin",ymin);
	  ymax = cfg.value("gfs","ymax",ymax);
	  delta =  cfg.value("gfs","delta",delta);
	  maxrange = cfg.value("gfs","maxrange",maxrange);
	  maxUrange = cfg.value("gfs","maxUrange",maxUrange);
	  regscore = cfg.value("gfs","regscore",regscore);
	  critscore = cfg.value("gfs","critscore",critscore);
	  kernelSize = cfg.value("gfs","kernelSize",kernelSize);
	  sigma = cfg.value("gfs","sigma",sigma);
	  iterations = cfg.value("gfs","iterations",iterations);
	  lstep = cfg.value("gfs","lstep",lstep);
	  astep = cfg.value("gfs","astep",astep);
	  maxMove = cfg.value("gfs","maxMove",maxMove);
	  srr = cfg.value("gfs","srr", srr);
	  srt = cfg.value("gfs","srt", srt);
	  str = cfg.value("gfs","str", str);
	  stt = cfg.value("gfs","stt", stt);
	  particles = cfg.value("gfs","particles",particles);
	  angularUpdate = cfg.value("gfs","angularUpdate", angularUpdate);
	  linearUpdate = cfg.value("gfs","linearUpdate", linearUpdate);
	  lsigma = cfg.value("gfs","lsigma", lsigma);
	  ogain = cfg.value("gfs","lobsGain", ogain);
	  lskip = (int)cfg.value("gfs","lskip", lskip);
	  //	  randseed = cfg.value("gfs","randseed", randseed);
	  resampleThreshold = cfg.value("gfs","resampleThreshold", resampleThreshold);
	  generateMap = cfg.value("gfs","generateMap", generateMap);

	  cerr << "Parameters parsed from " << configfilename << endl;
	}
	else {
		cerr << "Default parameters" << endl;
	}
	

	// init our lase
	int beams = 181;
	double resolution = 2*3.14/360; // one degree in radians
	OrientedPoint location = OrientedPoint(0,0,0);
	double span = 180;
	double max_range = 80;
	frontLaser = new RangeSensor("FLASER",beams,resolution,location,span,max_range);

	frontLaser->updateBeamsLookup();
	sensorMap.insert(make_pair("FLASER", frontLaser));

	//CREATION
	
	processor = new GridSlamProcessor;
		
	//SENSOR MAP 
	processor->setSensorMap(sensorMap);

	//set the command line parameters
	processor->setMatchingParameters(maxUrange, maxrange, sigma, kernelSize, lstep, astep, iterations, lsigma, ogain, lskip);
	processor->setMotionModelParameters(srr, srt, str, stt);
	processor->setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold);
	processor->setgenerateMap(generateMap);
	OrientedPoint initialPose(xmin+xmax/2, ymin+ymax/2, 0);
	
	
	//INITIALIZATION
	processor->init(particles, xmin, ymin, xmax, ymax, delta, initialPose);
	if (outfilename.length()>0)
		processor->outputStream().open(outfilename.c_str());
	
	GridSlamProcessor* ap, *copy=processor->clone();
	ap=processor; processor=copy; copy=ap;

}


Map<double, DoubleArray2D, false>* GFSJ2B2::updateMap(MaCI::Ranging::TDistanceArray& array, 
                                                      SLAM::RobotLocation& loc) {
	
	// transform into gmapping compatible form
	const RangeSensor* rs = frontLaser;
	RangeReading reading(rs, 0);
	reading.resize(rs->beams().size());
	for (unsigned int i = 0; i < 181; i++) {
		const MaCI::Ranging::TDistance& measurement = array[i];
		reading[i]=(double)measurement.distance;
	}
	reading.setPose(OrientedPoint(loc.x*10, loc.y*10, loc.theta));

	// try processing
	bool processed = processor->processScan(reading);

	Map<double, DoubleArray2D, false>* mymap;
	
	// this returns true when the algorithm effectively 
	// processes (the traveled path since the last 
	// processing is over a given threshold)
	if (processed){
		cerr << "PROCESSED" << endl;
		// for searching for the BEST PARTICLE INDEX
		unsigned int best_idx = processor->getBestParticleIndex();
		
		// if you want to access to the PARTICLE VECTOR
		// const GridSlamProcessor::ParticleVector& particles = processor->getParticles(); 
		// remember to use a const reference, otherwise 
		// it copies the whole particles and maps
		
		// this is for recovering the tree of PARTICLE TRAJECTORIES 
		// (obtaining the ancestor of each particle)
		// cerr << "Particle reproduction story begin" << endl;
		// for (unsigned int i=0; i<particles.size(); i++){
		// 	cerr << particles[i].previousIndex << "->"  << i << " ";
		// }
		// cerr << "Particle reproduction story end" << endl;
				
		// then if you want to access the BEST MAP,
		// of course by copying it in a plain structure 
		// at this point mymap is yours. Can do what you want.
		// double best_weight=particles[best_idx].weightSum;
		// cerr << "Best Particle is " << best_idx << " with weight " << best_weight << endl;
			
		mymap = processor->getParticles()[best_idx].map.toDoubleMap();

		// clone processor for some reason
		GridSlamProcessor* newProcessor = processor->clone();
		delete processor;
		processor = newProcessor;

	} 
	else {

		mymap = 0;

	}

	return mymap;
}

};
