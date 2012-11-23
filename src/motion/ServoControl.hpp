#include "MachineCtrlClient.hpp"


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

	ServoControl(MaCI::JointGroupCtrl::CJointGroupCtrlClient& servoCtrl);

	void Container(float);
	float getPosition(ServoControl::EServo);
	bool setPosition(ServoControl::EServo, float);


private:
	ServoControl(const ServoControl&); // unimplemented
	ServoControl operator=(const ServoControl&); // unimplemented
	MaCI::JointGroupCtrl::CJointGroupCtrlClient& servoCtrl;
};

}

#endif /* SERVOCONTROL_HPP_ */
