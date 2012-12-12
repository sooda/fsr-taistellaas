#ifndef MOTIONCONTROL_HPP_
#define MOTIONCONTROL_HPP_

#include "J2B2-API.hpp"
#include "SLAM/SLAM.hpp"
#include "SDL.h"
#include <list>
#include <vector>

namespace Motion {

const float SAFETY_DIST = 0.4;

class MotionControl {
public:
	MotionControl(CJ2B2Client &interface);
	~MotionControl();

	void setSpeed(float speed, float angle);
	void refreshSpeed(void);

	bool iterate(SLAM::RobotLocation current);
	void backOff();
	void stopBackingOff();

	typedef SLAM::RobotLocation Pose;
	typedef std::list<SLAM::Location> LocList;

	void setRoute(const LocList& route);
	void drawMap(SDL_Surface* screen, int sx, int sy) const;
	void setPose(Pose pose);
	struct Ctrl;
	Ctrl getCtrl() { return ctrl; }
	void setCtrl(float speed, float angle) { ctrl.speed = speed; ctrl.angle = angle; }
	void stop();
	bool running() const;
	void drawInfo(SDL_Surface* screen, int x, int y) const;
	bool backFromGoal(SLAM::RobotLocation current);
	bool rollStart(Pose pose);
	float routeLeft() const;
private:
	MotionControl(const MotionControl&);
	MotionControl& operator=(const MotionControl&);

	static const float acceleration;

	CJ2B2Client &interface;

	struct Ctrl {
		float speed, angle;
	} ctrl;

	struct {
		float p, a, iiris;
		float closeEnough;
		float closeEnoughLast;
	} k;

	// contains always also the one we're currently heading to,
	// thus, the front is popped off only after finishing with it
	LocList midpoints;
	Pose startPoint;

	struct ArcParams {
		float a, b, ox, oy; // it's fine for a or b to be negative
		// a is on x axis of the ellipse coordinate system, b on y axis
		// ox and oy are ellipse coordinate system origin in world coordinates
		bool horizontal;
	};

	bool nextMidpoint();
	void buildMidpoints(ArcParams ellipse);
	Pose lastPose;
	bool routeStarting;

	// TODO: a nice log of all control settings -- alpha, theta and shit
	struct HistPoint { Pose pose; Ctrl ctrl; float alpha, ata; };
	std::vector<HistPoint> history;
};

}

#endif
