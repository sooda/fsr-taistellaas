/*
 * ServoControl.cpp
 *
 *  Created on: 28 Oct 2012
 *      Author: Toni Liski
 */

#include "ServoControl.hpp"

using namespace Motion;

ServoControl::ServoControl(MaCI::JointGroupCtrl::CJointGroupCtrlClient& servoCtrl) : servoCtrl(servoCtrl)
{
}

void ServoControl::Container(float angle)
{

#if 0
	if (angle > M_PI/2) angle = M_PI/2;
	else if (angle < 0) angle = 0;
#endif
	
	int success_sent = servoCtrl.SetPosition(angle, KServoUserServo_1); //Ch 3
	
	// Check if was succesfully sent and print
	if (success_sent) {
		float pos;
		servoCtrl.GetPosition(KServoUserServo_1, pos, 500); //Servo, variable, timeout
		
		dPrint(1,"Container opening to %.3f", pos);

	
	} else {
		// Error
		dPrint(1, "fail!");
	}
}

float ServoControl::getPosition(ServoControl::EServo servo)
{
	float pos;

	servoCtrl.GetPosition(servo, pos, 500);
	return pos;

}

bool ServoControl::setPosition(ServoControl::EServo servo, float pos)
{
	servoCtrl.SetPosition(pos, servo);
	return true;
}
