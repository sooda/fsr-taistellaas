#include "motioncontrol.hpp"
#include "util.hpp"
#include <iostream>
#include <stdexcept>

using namespace std; // FIXME: remove, after fixing the debug prints
// (TODO: write a cool modern c++-raii-style logging mechanism with modular and named levels)
namespace Motion {

MotionControl::MotionControl(CJ2B2Client &interface) : interface(interface), ctrl(), k(), lastPose(0, 0, 0), routeStarting(0) {
#if 0
	// simulator
	const float max_speed=0.1;
	const float max_possible_dist=1; // no waypoint >1m away
	k.p = 3*max_speed/max_possible_dist;
	k.a = 3*0.32;
	k.b = 3*0.2;
#else
	k.p = 0.1;
	k.a = 0.3;
	k.b = 0.0; // 0.3 (0.1 in sim)
	k.iiris = k.p / M_PI;
#endif
	k.closeEnough = 0.15;
	interface.iMotionCtrl->SetStop();
	interface.iBehaviourCtrl->SetStart();
}

MotionControl::~MotionControl() {
	interface.iMotionCtrl->SetStop();
	cout << "--- BEGIN CTRL HISTORY ---" << endl;
	int i = 0;
	for (auto it = history.begin(); it != history.end(); ++it) {
		cout << i << " " << it->pose.x << " " << it->pose.y << " " << it->pose.theta << " " << it->ctrl.speed << " " << it->ctrl.angle << endl;
		i++;
	}
	cout << "--- END CTRL HISTORY ---" << endl;
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

// TODO: decide on how the angles work so that we don't need this shit
float fixangles(float ang) {
	if (ang > M_PI) return ang - 2*M_PI;
	if (ang < -M_PI) return ang + 2*M_PI;
	return ang; // hope so
}

// FIXME: initial position before navigating?
void MotionControl::setPose(Pose pose) {
	lastPose = pose;
}

// Main control loop to read current pose and update control values
// return false if no destination available (i.e. routing finished and need a new endpoint)
bool MotionControl::iterate(SLAM::RobotLocation myPose) {
	//if (myPose.theta < 0) myPose.theta += 2*M_PI;
	// you spin me right round
	cout << "hihii " << myPose.theta;
	while (myPose.theta > M_PI)
		myPose.theta -= 2*M_PI;
	while (myPose.theta < -M_PI)
		myPose.theta += 2*M_PI;
	lastPose = myPose;
	cout << " " << myPose.theta << endl;

	cout << "ITERATE" << endl;
	cout << midpoints.size() << endl;
	if (midpoints.size() == 0) { // FIXME: kludge
		cout << "\tNo midpoints" << endl;
		ctrl.speed = 0;
		ctrl.angle = 0;
		return true;
	}
	Pose midpdest = currentMidpoint();
	float dx = midpdest.x - myPose.x;
	float dy = midpdest.y - myPose.y;
	// TODO: subtract mod2pi somehow so that almost2pi-littleover0=2pi-almost2pi+littleover0 (?)
	//if (myPose.theta<0.01)myPose.theta=2*M_PI;
	float dt = midpdest.theta - myPose.theta; // my_t-dest_t in the lecture slide
	dt = fixangles(dt);
	float rho = sqrt(dx * dx + dy * dy);
	float ata = atan2(dy, dx);
	//if (ata < 0) ata += 2*M_PI;
	float alpha = ata - myPose.theta;
	alpha = fixangles(alpha);
	float beta = (dt - alpha);
	beta = fixangles(beta);

	// When new route is gotten, first rotate to orientation of first waypoint.
	static const float eps = 20*M_PI/180;
	if (routeStarting) {
		if (floateq(myPose.theta, midpdest.theta, eps)) {
			routeStarting = 0;
			return nextMidpoint();
		}
		else {
			ctrl.speed = 0;
			ctrl.angle = k.a * dt;
			return true;
		}
	}
	ctrl.speed = k.p - k.iiris * fabsf(alpha);// * rho;
	ctrl.angle = k.a * alpha - k.b * beta;

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
	history.push_back(HistPoint{lastPose, Ctrl{ctrl.speed, ctrl.angle}});
	if (rho < k.closeEnough) {
		cout << "\tCLOSE ENOUGH!" << endl;
		return nextMidpoint();
	}
	return true;
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
	for (auto it = poles.begin(); it != poles.end(); ++it) {
		filledCircleRGBA(screen, sx + it->x * pxl_per_m, sy - it->y * pxl_per_m, 5, 255, 0, 0, 255);
	}
	for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
		filledCircleRGBA(screen, sx + it->x * pxl_per_m, sy - it->y * pxl_per_m, 5, 0, 0, 255, 255);
	}
	auto last = midpoints.begin();
	//cout << endl << "Starting le draw" << endl;
	if (last != midpoints.end()) {
		int i=1;
		auto next = last;
		lineRGBA(screen, sx + lastPose.x * pxl_per_m, sy - lastPose.y * pxl_per_m, sx + last->x * pxl_per_m, sy - last->y * pxl_per_m, 255, 255, 0, 255);
		for (++next; ; ++next) {
			float arrowx = cos(last->theta), arrowy = sin(last->theta);
			lineRGBA(screen, sx + last->x * pxl_per_m, sy - last->y * pxl_per_m, sx + (last->x+arrowx) * pxl_per_m, sy - (last->y+arrowy) * pxl_per_m, 255-255*i/midpoints.size(), 255*i/midpoints.size(), 0, 255);

			if (next == midpoints.end())
				break;

			lineRGBA(screen, sx + last->x * pxl_per_m, sy - last->y * pxl_per_m, sx + next->x * pxl_per_m, sy - next->y * pxl_per_m, 255, 255, 255, 255);
			last = next;
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
	// not initialized if 0
	if (ellipse.ox != 0 && ellipse.oy != 0) {
		buildMidpoints(ellipse);
	}
}
// XXX this should maybe not be called from another thread while doing some parsing
// Currently only called once after starting, will probably be called after having
// finished the whole route
void MotionControl::setRoute(const PoseList& route) {
	waypoints = route;
	dPrint(0, "reload waypoint from current location");
	reloadWaypoint(lastPose);
	routeStarting = 1;
}

// Rebuild the midpoint list from the arc parameter specification
void MotionControl::buildMidpoints(ArcParams ellipse) {
	assert((midpoints.empty() && "One does not simply build midpoints while still old points available"));
	cout << "Build midpoints." << endl;
	int steps = 6 * sqrt(ellipse.a * ellipse.a + ellipse.b * ellipse.b);
	const float step = M_PI / 2 / steps;
	for (float t = 0; t < M_PI / 2 + step*0.5; t += step) {
		cout << "\tMidpoint " << t << endl;
		float x, y, dx, dy;
		if (!ellipse.horizontal) {
			x = ellipse.ox + ellipse.a * cos(t); // + ox
			y = ellipse.oy + ellipse.b * sin(t); // + oy
			dx = ellipse.a * -sin(t);
			dy = ellipse.b * cos(t);
		} else {
			x = ellipse.ox + ellipse.a * sin(t); // + ox
			y = ellipse.oy + ellipse.b * cos(t); // + oy
			dx = ellipse.a * cos(t);
			dy = ellipse.b * -sin(t);
		}
		float theta = atan2(dy, dx);
		//if (theta < 0) theta += 2*M_PI;
		cout << "\t\tAt pos " << x << " " << y << endl;
		cout << "\t\tAt angle " << theta << endl;
		midpoints.push_back(SLAM::RobotLocation(x, y, theta));
	}
	if (!routeStarting)
		nextMidpoint(); // FIXME -- this erases the first from the list, we're on that when starting..
}

// Derive an ellipse arc to drive on, from current location to a destination
MotionControl::ArcParams MotionControl::ellipseParams(Pose source, Pose dest) {
	cout << "ellipse params from " << source.x << " " << source.y << " " << source.theta << " to " << dest.x << "," << dest.y << "," << dest.theta << endl;
	ArcParams p;
	static const float eps = 20*M_PI/180;
	if (floateq(source.theta, DIR_UP, eps) || floateq(source.theta, DIR_DOWN, eps)) {
		cout << "\tVertical" << endl;
		// facing up or down
		p.ox = dest.x;
		p.oy = source.y;
		p.a = source.x - dest.x;
		p.b = dest.y - source.y;
		p.horizontal = false;
	} else if (floateq(source.theta, DIR_RIGHT, eps) || floateq(source.theta, DIR_RIGHT + 2*M_PI) || floateq(source.theta, DIR_LEFT, eps)) { // FIXME: hack for both 0 and 2pi
		cout << "\tHorizontal" << endl;
		p.ox = source.x;
		p.oy = dest.y;
		p.a = dest.x - source.x;
		p.b = source.y - dest.y;
		p.horizontal = true;
	} else {
		dPrint(0, ("FIXME: cannot continue -- bad pose for waypoint " + lexical_cast(source.theta)).c_str());
		p.ox=p.oy=0; // invent a better way for this
	}
	cout << "Got params: " << p.ox << " " << p.oy << " " << p.a << " " << p.b << endl;
	return p;
}

// Remove the current midpoint and update target waypoint to be the next one
// Go to next waypoint and update midpoint list if all midpoints have been consumed
// return false if no midpoint available
bool MotionControl::nextMidpoint(void) {
midpoints.pop_front();
	if (midpoints.empty()) {
		Pose lastWaypoint = waypoints.front();
		waypoints.pop_front();
		if (waypoints.size()) {
			dPrint(0, "reload new waypoint");
			reloadWaypoint(lastWaypoint);
		} else {
			// finished with current route -- cannot continue
			return false;
		}
	}
	return true;
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
