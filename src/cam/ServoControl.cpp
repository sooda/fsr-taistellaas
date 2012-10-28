/*
 * ServoControl.cpp
 *
 *  Created on: 28 Oct 2012
 *      Author: Toni Liski
 */

#include "ServoControl.hpp"

using namespace cam;

ServoControl::ServoControl(MaCI::MachineCtrl::CMachineCtrlClient *machine)
	: iMachine(machine), iServoCtrl(NULL)
{
	// Get component list
	MaCI::MachineCtrl::TMaCIMachineComponents comp;
	iMachine->GetMachineComponents(comp);

	// If the list has any components ...
	if (comp.size() > 0) {
		iServoCtrl = iMachine->GetJointGroupCtrlClient("ServoCtrl", true);
	}
	else {
		// No components! Propably error connecting.
		dPrintLCRed(ODERROR,"WARNING: No services found with given parameters!");
	}
}

ServoControl::~ServoControl()
{
	// TODO Auto-generated destructor stub
}

void ServoControl::TestMovement()
{
	float ptu_pan = 0.0;
	float ptu_tilt = 0.0;
	float ptu_pan_delta = 0.0;
	float ptu_tilt_delta = 0.0;

//	float M_PI =

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
			dPrint(3,"Camera now pointing to pan:%.3f, tilt:%.3f", ppos, tpos);

			// Wait to stabilize
			// ownSleep_ms(200);
		} else {
			// Some failure?
		}
    }

}


