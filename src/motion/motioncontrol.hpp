#include "../J2B2-API.hpp"
#ifndef MOTIONCONTROL_HPP_
#define MOTIONCONTROL_HPP_

namespace Motion {

const float SAFETY_DIST = 0.4;

class MotionControl {
	public:
		MotionControl(CJ2B2Client &interface);
		~MotionControl();
		/* This will eventually get removed, as we don't want to control the
		 * speed and angle directly but instead give a list of points to reach
		 * or something; TBD.
		 */
		void setSpeed(float speed, float angle);
		void avoidObstacle(float obstacleAngle);
	private:
		CJ2B2Client &interface;
		static const float acceleration;
		MotionControl(const MotionControl&);
		MotionControl& operator=(const MotionControl&);
};

}

#endif
