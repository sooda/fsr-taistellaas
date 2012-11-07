#include "motioncontrol.hpp"
#include "util.hpp"
#include <iostream>
#include <stdexcept>

using namespace std; // FIXME: remove, after fixing the debug prints
// (TODO: write a cool modern c++-raii-style logging mechanism with modular and named levels)
namespace Motion {

MotionControl::MotionControl(CJ2B2Client &interface) : interface(interface), ctrl(), k(), lastPose(0, 0, 0) {
	const float max_speed=0.1;
	const float max_possible_dist=1; // no waypoint >1m away
	k.p = 4*max_speed/max_possible_dist;
	k.a = 4*0.32;
	k.b = 4*0.2;
	k.closeEnough = 0.05;
	interface.iMotionCtrl->SetStop();
	interface.iBehaviourCtrl->SetStart();
}

MotionControl::~MotionControl() {
	interface.iMotionCtrl->SetStop();
}

// Set directional and angular speed to the robot controller immediately
// This should get called, in a way or other, about in 200 ms intervals
void MotionControl::setSpeed(float speed, float angle) {
	interface.iMotionCtrl->SetSpeed(speed, angle, acceleration);
}

// Send the current wanted speed (from the automatic control) to the robot controller
void MotionControl::refreshSpeed(void) {
	setSpeed(ctrl.speed, ctrl.angle);
}

// FIXME: initial position before navigating?
void MotionControl::setPose(Pose pose) {
	lastPose = pose;
}

float r2d(float deg) { return deg / M_PI * 180; }

// Main control loop to read current pose and update control values
void MotionControl::iterate(SLAM::RobotLocation myPose) {
	if (myPose.theta < 0) myPose.theta += 2*M_PI;
	lastPose = myPose;
	if (waypoints.size() == 0) { // FIXME: kludge
		ctrl.speed = 0;
		ctrl.angle = 0;
		return;
	}
	Pose midpdest = currentMidpoint();
	float dx = midpdest.x - myPose.x;
	float dy = midpdest.y - myPose.y;
	// TODO: subtract mod2pi somehow so that almost2pi-littleover0=2pi-almost2pi+littleover0 (?)
	if (myPose.theta<0.01)myPose.theta=2*M_PI;
	float dt = midpdest.theta - myPose.theta; // my_t-dest_t in the lecture slide
	float rho = sqrt(dx * dx + dy * dy);
	float ata = atan2(dy, dx);
	if (ata < 0) ata += 2*M_PI;
	float alpha = ata - myPose.theta;
	float beta = (dt - alpha);

	ctrl.speed = k.p * rho;
	ctrl.angle = k.a * alpha - k.b * beta;

	cout << "ITERATE" << endl;
	cout<< "\tdx = " << dx << endl;
	cout<< "\tdy = " << dy << endl;
	cout<< "\tmy theta = " << myPose.theta << " " << r2d(myPose.theta) << endl;
	cout<< "\tdest theta = " << midpdest.theta << " " << r2d(midpdest.theta) << endl;
	cout<< "\tatan2 dydx = " << ata << " " << r2d(ata) << endl;
	cout<< "\tdt = " << dt << " " << r2d(dt) << endl;
	cout<< "\trho = " << rho << endl;
	cout<< "\talpha = " << alpha << " " << r2d(alpha) << endl;
	cout<< "\tbeta = " << beta << " " << r2d(beta) << endl;
	cout<< "\tv = " << ctrl.speed << endl;
	cout<< "\tw = " << ctrl.angle << " " << r2d(ctrl.angle) << endl;

	if (rho < k.closeEnough) {
		cout << "\tCLOSE ENOUGH!" << endl;
		nextMidpoint();
	}
}


void MotionControl::drawMap(SDL_Surface* screen, int sx, int sy) const {
	//pixelRGBA(screen, sx, sy, 255, 255, 255, 255);
	lineRGBA(screen, sx, sy, sx, sy-100, 255, 255, 0, 255);
	lineRGBA(screen, sx, sy, sx+100, sy, 255, 255, 0, 255);
	float pxl_per_m = 100;
	filledCircleRGBA(screen, sx + lastPose.x * pxl_per_m, sy - lastPose.y * pxl_per_m, 5, 0, 255, 255, 255);
	lineRGBA(screen, sx + lastPose.x * pxl_per_m, sy - lastPose.y * pxl_per_m, sx + (lastPose.x + cos(lastPose.theta)) * pxl_per_m, sy - (lastPose.y + sin(lastPose.theta)) * pxl_per_m, 255, 0, 0, 255);
	PoseList poles{
		Pose(3, 3.1, 0),
		Pose(3, 2.3, 0),

		Pose(3.2, 1.2, 0),
		Pose(4, 1.2, 0),

		Pose(2.2, 1.9, 0),
		Pose(2.2, 1.1, 0),

		Pose(1, 0.3, 0),
		Pose(1, 1.1, 0),

		Pose(1.5, 2.8, 0),
		Pose(1.5, 2, 0)

	};
	for (const Pose& p: poles) {
		filledCircleRGBA(screen, sx + p.x * pxl_per_m, sy - p.y * pxl_per_m, 5, 255, 0, 0, 255);
	}
	for (const Pose& p: waypoints) {
		filledCircleRGBA(screen, sx + p.x * pxl_per_m, sy - p.y * pxl_per_m, 5, 0, 0, 255, 255);
	}
	auto last = midpoints.begin();
	//cout << endl << "Starting le draw" << endl;
	if (last != midpoints.end()) {
		int i=1;
		auto it = last;
		lineRGBA(screen, sx + lastPose.x * pxl_per_m, sy - lastPose.y * pxl_per_m, sx + last->x * pxl_per_m, sy - last->y * pxl_per_m, 255, 255, 0, 255);
		for (++it; it != midpoints.end(); ++it) {
			//cout << "Le line: " << sx + last->x * pxl_per_m << " " << sy + last->y * pxl_per_m << " " << sx + it->x * pxl_per_m << " " << sy + it->y * pxl_per_m << endl;
			lineRGBA(screen, sx + last->x * pxl_per_m, sy - last->y * pxl_per_m, sx + it->x * pxl_per_m, sy - it->y * pxl_per_m, 255, 255, 255, 255);
			float arrowx = cos(last->theta), arrowy = sin(last->theta);

			lineRGBA(screen, sx + last->x * pxl_per_m, sy - last->y * pxl_per_m, sx + (last->x+arrowx) * pxl_per_m, sy - (last->y+arrowy) * pxl_per_m, 255-255*i/midpoints.size(), 255*i/midpoints.size(), 0, 255);
			last = it;
			i++;
		}

	}
}

// The midpoint we're heading towards currently
const MotionControl::Pose& MotionControl::currentMidpoint(void) const {
	return midpoints.front();
}

// Call after updating waypoint list or after having reached one waypoint to get the next one
// Calculates the route settings till next waypoint
void MotionControl::reloadWaypoint(Pose source) {
	auto destination = waypoints.front();
	ArcParams ellipse = ellipseParams(source, destination);
	buildMidpoints(ellipse);
}
// XXX this should maybe not be called from another thread while doing some parsing
// Currently only called once after starting, will probably be called after having
// finished the whole route
void MotionControl::setRoute(const PoseList& route) {
	waypoints = route;
	dPrint(0, "reload waypoint from current location");
	reloadWaypoint(lastPose);
}

// Rebuild the midpoint list from the arc parameter specification
void MotionControl::buildMidpoints(ArcParams ellipse) {
	assert((midpoints.empty() && "One does not simply build midpoints while still old points available"));
	cout << "Build midpoints." << endl;
	const int steps = 6;
	if (!ellipse.horizontal) {
		const float step = M_PI / 2 / steps;
		for (float t = 0; t < M_PI / 2 + step*0.5; t += step) {
			cout << "\tMidpoint " << t << endl;
			float x = ellipse.ox + ellipse.a * cos(t); // + ox
			float y = ellipse.oy + ellipse.b * sin(t); // + oy
			float dx = ellipse.a * -sin(t);
			float dy = ellipse.b * cos(t);
			float theta = atan2(dy, dx);
			if (theta < 0) theta += 2*M_PI;
			cout << "\t\tAt pos " << x << " " << y << endl;
			cout << "\t\tAt angle " << theta << endl;
			midpoints.push_back(SLAM::RobotLocation(x, y, theta));
		}
	} else {
		const float step = M_PI / 2 / steps;
		for (float t = 0; t < M_PI / 2 + step*0.5; t += step) {
			cout << "\tMidpoint " << t << endl;
			float x = ellipse.ox + ellipse.a * sin(t); // + ox
			float y = ellipse.oy + ellipse.b * cos(t); // + oy
			float dx = ellipse.a * cos(t);
			float dy = ellipse.b * -sin(t);
			float theta = atan2(dy, dx);
			if (theta < 0) theta += 2*M_PI;
			cout << "\t\tAt pos " << x << " " << y << endl;
			cout << "\t\tAt angle " << theta << endl;
			midpoints.push_back(SLAM::RobotLocation(x, y, theta));
		}
	}
	nextMidpoint(); // FIXME -- this erases the first from the list, we're on that when starting..
}

// Derive an ellipse arc to drive on, from current location to a destination
MotionControl::ArcParams MotionControl::ellipseParams(Pose source, Pose dest) {
	cout << "ellipse params from " << source.x << " " << source.y << " " << source.theta << " to " << dest.x << "," << dest.y << "," << dest.theta << endl;
	ArcParams p;
	static const float eps = 0.01;
	if (fabsf(source.theta - M_PI/2) < eps || fabs(source.theta - 3*M_PI/4) < eps) {
		cout << "\tVertical" << endl;
		// facing up or down
		p.ox = dest.x;
		p.oy = source.y;
		p.a = source.x - dest.x;
		p.b = dest.y - source.y;
		p.horizontal = false;
	} else if (fabsf(source.theta - 0) < eps || fabsf(source.theta - 2*M_PI) < eps || fabs(source.theta - M_PI) < eps) {
		cout << "\tHorizontal" << endl;
		p.ox = source.x;
		p.oy = dest.y;
		p.a = dest.x - source.x;
		p.b = source.y - dest.y;
		p.horizontal = true;
	} else {
		throw std::runtime_error("FIXME: bad pose for waypoint " + lexical_cast(source.theta));
	}
	cout << "Got params: " << p.ox << " " << p.oy << " " << p.a << " " << p.b << endl;
	return p;
}

// Remove the current midpoint and update target waypoint to be the next one
// Go to next waypoint and update midpoint list if all midpoints have been consumed
void MotionControl::nextMidpoint(void) {
	midpoints.pop_front();
	if (midpoints.empty()) {
		Pose lastWaypoint = waypoints.front();
		waypoints.pop_front();
		if (waypoints.size()) {
			dPrint(0, "reload new waypoint");
			reloadWaypoint(lastWaypoint);
		} else {
			// TODO: inform the main planner
			// (note: tried to get next, not yet finished)
		}
	}
}

// TODO
void MotionControl::avoidObstacle(float obstacleAngle) {
	dPrint(1, "Too close -- avoiding obstacle");
	// just go back as fast as possible.
	// TODO: reverse a bit, turn 90 degrees and go forward
	interface.iMotionCtrl->SetSpeed(-1, 0, acceleration);
	// interface.iMotionCtrl->SetStop();
}

const float MotionControl::acceleration = 0.3;

}
