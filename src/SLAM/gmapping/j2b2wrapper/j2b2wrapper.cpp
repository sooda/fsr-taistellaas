/*****************************************************************
 *
 * This file is part of the GMAPPING project
 *
 * GMAPPING Copyright (c) 2004 Giorgio Grisetti, 
 * Cyrill Stachniss, and Wolfram Burgard
 *
 * This software is licensed under the "Creative Commons 
 * License (Attribution-NonCommercial-ShareAlike 2.0)" 
 * and is copyrighted by Giorgio Grisetti, Cyrill Stachniss, 
 * and Wolfram Burgard.
 * 
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/2.0/
 * 
 * GMAPPING is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/


#include "j2b2wrapper.hpp"

using namespace GMapping;
using namespace std;

SensorMap J2B2Wrapper::m_sensorMap;
deque<RangeReading> J2B2Wrapper::m_rangeDeque;
pthread_mutex_t J2B2Wrapper::m_mutex;  
sem_t J2B2Wrapper::m_dequeSem;  
pthread_mutex_t J2B2Wrapper::m_lock;  
pthread_t J2B2Wrapper::m_readingThread;
RangeSensor* J2B2Wrapper::m_frontLaser=0;
bool J2B2Wrapper::m_threadRunning=false;
OrientedPoint J2B2Wrapper::m_truepos;
bool J2B2Wrapper::stopped=true;


bool J2B2Wrapper::start(){
	if (m_threadRunning)
		return false;

	// thread stuff
	pthread_mutex_init(&m_mutex, 0);
	pthread_mutex_init(&m_lock, 0);
	sem_init(&m_dequeSem, 0, 0);
	m_threadRunning=true;
	pthread_create (&m_readingThread,0,m_reading_function,0);

	// init our laser
	m_frontLaser=new RangeSensor("UBERLASER",181,1,OrientedPoint(0,0,0),0,100);
	m_frontLaser->updateBeamsLookup();
	m_sensorMap.insert(make_pair(sensorName, m_frontLaser));

	return true; 
}

void J2B2Wrapper::lock(){
	pthread_mutex_lock(&m_lock);
}

void J2B2Wrapper::unlock(){
	pthread_mutex_unlock(&m_lock);
}


bool J2B2Wrapper::sensorMapComputed(){
	pthread_mutex_lock(&m_mutex);
	bool smok=m_frontLaser;
	pthread_mutex_unlock(&m_mutex);
	return smok;
}

const SensorMap& J2B2Wrapper::sensorMap(){
	return m_sensorMap;
}
 
bool J2B2Wrapper::isRunning(){
	return m_threadRunning;
}

bool J2B2Wrapper::isStopped(){
	return stopped;
}

int J2B2Wrapper::queueLength(){
	int ql=0;
	pthread_mutex_lock(&m_mutex);
	ql=m_rangeDeque.size();
	pthread_mutex_unlock(&m_mutex);
	return ql;
}

OrientedPoint J2B2Wrapper::getTruePos(){
	return m_truepos;
}

bool J2B2Wrapper::getReading(RangeReading& reading){
	bool present=false;
	sem_wait(&m_dequeSem);
	pthread_mutex_lock(&m_mutex);
	if (!m_rangeDeque.empty()){
		reading=m_rangeDeque.front();
		m_rangeDeque.pop_front();
		present=true;
	}
	int sval;
	sem_getvalue(&m_dequeSem,&sval);
	pthread_mutex_unlock(&m_mutex);
	return present;
}

void J2B2Wrapper::addReading(RangeReading& reading){
	pthread_mutex_lock(&m_mutex);
	m_rangeDeque.push_back(reading);
	pthread_mutex_unlock(&m_mutex);
	sem_post(&m_dequeSem);
	int sval;
	sem_getvalue(&m_dequeSem,&sval);
}


void J2B2Wrapper::robot_frontlaser_handler(carmen_robot_laser_message* frontlaser) {
	RangeReading reading=carmen2reading(*frontlaser);
	addReading(reading);
}


void J2B2Wrapper:: navigator_go_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) {
  cerr<<"go"<<endl;
  stopped=false;
}


void J2B2Wrapper:: navigator_stop_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) {
  cerr<<"stop"<<endl;
  stopped=true;
}



void J2B2Wrapper::simulator_truepos_handler(carmen_simulator_truepos_message* truepos){
	m_truepos.x=truepos->truepose.x;
	m_truepos.y=truepos->truepose.y;
	m_truepos.theta=truepos->truepose.theta;
}

/*
 * Make readings from robot data
 */
RangeReading J2B2Wrapper::robot2reading(const MaCI::Ranging::TDistanceArray& array, RobotLocation loc){
	const RangeSensor* rs = m_frontLaser;
	RangeReading reading(rs, 0);
	reading.resize(rs->beams().size());
	for (unsigned int i = 0; i < 181; i++) {
		const MaCI::Ranging::TDistance& measurement = array[i];
		reading[i]=(double)measurement.distance;
	}
	reading.setPose(OrientedPoint(loc.x, loc.y, loc.theta));
	return reading;
}


void * J2B2Wrapper::m_reading_function(void*){
	while (true) {
		lock();
		listen(100);
		unlock();
		usleep(20000);
	}    
	return 0;
}


