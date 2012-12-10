/*
 * ServoControl.cpp
 *
 *  Created on: 28 Oct 2012
 *      Author: Toni Liski
 */

#include "ServoControl.hpp"
# include <iostream>

using namespace Motion;

ServoControl::ServoControl(CJ2B2Client &interface)
	: interface(interface)
{
	iServoCtrl = interface.iServoCtrl;
}

void ServoControl::resetServos()
{
	float ptu_pan = 0.0;
	float ptu_tilt = -0.8;//-1.25;

    if (iServoCtrl)
    {
		// Do value checking and limiting.
		if (ptu_pan > M_PI/2) ptu_pan = M_PI/2;
		else if (ptu_pan < -M_PI/2) ptu_pan = -M_PI/2;
		if (ptu_tilt > M_PI/2) ptu_tilt = M_PI/2;
		else if (ptu_tilt < -M_PI/2) ptu_tilt = -M_PI/2;

		// Do control
		int r = this->setPosition(KServoCameraPTUPan, ptu_pan);
		r &= this->setPosition(KServoCameraPTUTilt, ptu_tilt);

		if (r) {
			float ppos,tpos;
			ppos = this->getPosition(KServoCameraPTUPan);
			tpos = this->getPosition(KServoCameraPTUTilt);
			std::cout << "Camera now pointing to pan:" << ppos << ", tilt:" << tpos << std::endl;
		} else {
			std::cout << "Camera control error" << std::endl;
		}
		
		// hatch control
		this->setPosition(KServoUserServo_0, 0.0);
		
    }
}

void ServoControl::setHatch(bool state) {
	setPosition(KServoUserServo_0, state ? 0 : 1);
}

float ServoControl::getPosition(ServoControl::EServo servo) const
{
	if (servopos.find(servo) == servopos.end())
		return 0.0;
	else
		return servopos.find(servo)->second;
}

bool ServoControl::setPosition(ServoControl::EServo servo, float pos)
{
	// Scale [-pi,pi] -> [-1,1]
	// don't ask why we have to scale it, we just have to
	iServoCtrl->SetPosition(pos/M_PI, servo);
	this->servopos[servo] = pos;
	return true;
}
