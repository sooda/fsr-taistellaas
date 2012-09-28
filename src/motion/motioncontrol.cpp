#include "motioncontrol.hpp"

namespace Motion {

MotionControl::MotionControl(CJ2B2Client &interface):interface(interface) {
	interface.iMotionCtrl->SetStop();
	interface.iBehaviourCtrl->SetStart();
}
MotionControl::~MotionControl() {
	interface.iMotionCtrl->SetStop();
}
void MotionControl::setSpeed(float speed, float angle) {
	interface.iMotionCtrl->SetSpeed(speed, angle, acceleration);
}
void MotionControl::avoidObstacle(float obstacleAngle) {
	dPrint(1, "Too close -- avoiding obstacle");
	// just go back as fast as possible.
	// TODO: reverse a bit, turn 90 degrees and go forward
	interface.iMotionCtrl->SetSpeed(-1, 0, acceleration);
	// interface.iMotionCtrl->SetStop();
}
const float MotionControl::acceleration = 0.2;

}
