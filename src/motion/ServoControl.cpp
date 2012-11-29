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

void ServoControl::TestMovement()
{
	float ptu_pan = 0.0;
	float ptu_tilt = 0.0;
	float ptu_pan_delta = 0.0;
	float ptu_tilt_delta = 0.0;

    if (iServoCtrl)
    {

    	// Do movement
		ptu_pan += ptu_pan_delta * M_PI/20;
		ptu_tilt += ptu_tilt_delta * M_PI/20;

		// Do value checking and limiting.
		if (ptu_pan > M_PI/2) ptu_pan = M_PI/2;
		else if (ptu_pan < -M_PI/2) ptu_pan = -M_PI/2;
		if (ptu_tilt > M_PI/2) ptu_tilt = M_PI/2;
		else if (ptu_tilt < -M_PI/2) ptu_tilt = -M_PI/2;

		// Do control
		int r = iServoCtrl->SetPosition(ptu_pan, KServoCameraPTUPan);
		r &= iServoCtrl->SetPosition(ptu_tilt, KServoCameraPTUTilt);

		if (r) {
			float ppos,tpos;
			iServoCtrl->GetPosition(KServoCameraPTUPan, ppos, 500);
			iServoCtrl->GetPosition(KServoCameraPTUTilt, tpos, 500);
			std::cout << "Camera now pointing to pan:" << ppos << ", tilt:" << tpos << std::endl;

			// Wait to stabilize
			// ownSleep_ms(200);
		} else {
			std::cout << "Camera control error" << std::endl;
			// Some failure?
		}
    }

}

float ServoControl::getPosition(ServoControl::EServo servo)
{
	if (!iServoCtrl) return 0.0;

	float pos;

	iServoCtrl->GetPosition(servo, pos, 10);
	return pos;

}

bool ServoControl::setPosition(ServoControl::EServo servo, float pos)
{
	if (iServoCtrl) return false;

	iServoCtrl->SetPosition(pos, servo);
	return true;
}
