#include "MachineCtrlClient.hpp"
#include "J2B2-API.hpp"


/*
 * ServoControl.hpp
 *
 *  Created on: 28 Oct 2012
 *      Author: Toni Liski
 */

#ifndef SERVOCONTROL_HPP_
#define SERVOCONTROL_HPP_

namespace Motion {

class ServoControl {
public:

	enum EServo {
	  KServoCameraPTUPan,     ///< Camera PTU - Pan
	  KServoCameraPTUTilt,    ///< Camera PTU - Tilt
	  KServoUserServo_0,      ///< User defined servo 0 (on Ch 2)
	  KServoUserServo_1,      ///< User defined servo 1 (on Ch 3)
	  KServoUserServo_2,      ///< User defined servo 2 (on Ch 4)
	  KServoUserServo_3       ///< User defined servo 3 (on Ch 5)
	};

	ServoControl(CJ2B2Client&);

	void TestMovement();
	float getPosition(ServoControl::EServo);
	bool setPosition(ServoControl::EServo, float);

	ServoControl operator=(const ServoControl&); // unimplemented

private:
	CJ2B2Client &interface;
	MaCI::JointGroupCtrl::CJointGroupCtrlClient *iServoCtrl;  ///< PTU of camera
};

}

#endif /* SERVOCONTROL_HPP_ */
