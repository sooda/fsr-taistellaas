#ifndef MOTIONCONTROL_HPP_
#define MOTIONCONTROL_HPP_

#include "J2B2-API.hpp"
#include "SLAM/SLAM.hpp"
#include "SDL.h"
#include <list>

namespace Motion {

const float SAFETY_DIST = 0.4;

class MotionControl {
public:
	MotionControl(CJ2B2Client &interface);
	~MotionControl();

	void setSpeed(float speed, float angle);
	void refreshSpeed(void);

	void iterate(SLAM::RobotLocation current);
	void avoidObstacle(float obstacleAngle);

	typedef SLAM::RobotLocation Pose;
	typedef std::list<Pose> PoseList;

	void setRoute(const PoseList& route);
	void drawMap(SDL_Surface* screen, int sx, int sy) const;
	void setPose(Pose pose) ;
private:
	MotionControl(const MotionControl&);
	MotionControl& operator=(const MotionControl&);

	static const float acceleration;

	CJ2B2Client &interface;

	struct {
		float speed, angle;
	} ctrl;

	struct {
		float p, a, b;
		float closeEnough;
	} k;

	// these lists contain always also the one we're currently heading to,
	// thus, the front is popped off only after finishing with it
	PoseList waypoints;
	PoseList midpoints;

	struct ArcParams {
		float a, b, ox, oy; // it's fine for a or b to be negative
		// a is on x axis of the ellipse coordinate system, b on y axis
		// ox and oy are ellipse coordinate system origin in world coordinates
		bool horizontal;
	};

	const Pose& currentMidpoint(void) const;
	void nextMidpoint();
	void reloadWaypoint(Pose src);
	static ArcParams ellipseParams(Pose source, Pose dest);
	void buildMidpoints(ArcParams ellipse);
	Pose lastPose;
};

}

#endif
