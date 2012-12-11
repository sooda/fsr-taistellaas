#include "motioncontrol.hpp"
#include "util.hpp"
#include <iostream>
#include <stdexcept>

using namespace std; // FIXME: remove, after fixing the debug prints
// (TODO: write a cool modern c++-raii-style logging mechanism with modular and named levels)
namespace Motion {

MotionControl::MotionControl(CJ2B2Client &interface) : interface(interface), ctrl(), k(), lastPose(0, 0, 0), routeStarting(false) {
	k.p = 0.05;
	k.a = 0.2;
	k.iiris = k.p / M_PI;
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

bool MotionControl::rollStart(Pose pose) {
	static bool rotated = false;
	if (floateq(pose.theta, M_PI, 20*M_PI/180))
		rotated = true;
	bool finished = rotated && floateq(pose.theta, 0, 20*M_PI/180);
	if (finished) {
		ctrl.angle = 0;
	} else {
		ctrl.angle = 1 / 10.0 * M_PI;
	}
	return !finished;
}

bool MotionControl::running() const {
	// TODO: write a lock for the midpoint vector to avoid race conditions with planner thread
	return !midpoints.empty();
}
void MotionControl::stop() {
	// FIXME: again a race condition, with control parameters (is a big lock for the whole class enough?)
	ctrl.speed = 0;
	ctrl.angle = 0;
	interface.iMotionCtrl->SetStop();
	midpoints.clear();
}

// Main control loop to read current pose and update control values
// return false if no destination available (i.e. routing finished and need a new endpoint)
bool MotionControl::iterate(SLAM::RobotLocation myPose) {
	//if (myPose.theta < 0) myPose.theta += 2*M_PI;
	// you spin me right round
	while (myPose.theta > M_PI)
		myPose.theta -= 2*M_PI;
	while (myPose.theta < -M_PI)
		myPose.theta += 2*M_PI;
	lastPose = myPose;
	cout << " " << myPose.theta << endl;

	cout << "ITERATE" << endl;
	cout << midpoints.size() << endl;
	if (midpoints.size() == 0) {
		cout << "\tNo midpoints" << endl;
		ctrl.speed = 0;
		ctrl.angle = 0;
		return false;
	}
	SLAM::Location midpdest = midpoints.front();
	float dx = midpdest.x - myPose.x;
	float dy = midpdest.y - myPose.y;
	// TODO: subtract mod2pi somehow so that almost2pi-littleover0=2pi-almost2pi+littleover0 (?)
	//if (myPose.theta<0.01)myPose.theta=2*M_PI;
	float rho = sqrt(dx * dx + dy * dy);
	float ata = atan2(dy, dx);
	//if (ata < 0) ata += 2*M_PI;
	float alpha = ata - myPose.theta;
	alpha = fixangles(alpha);

	// When new route is gotten, first rotate to orientation of first waypoint.
	static const float eps = 20*M_PI/180;
	if (routeStarting) {
		if (floateq(myPose.theta, startPoint.theta, eps)) {
			routeStarting = false;
			return nextMidpoint();
		} else {
			ctrl.speed = 0;
			float dt = startPoint.theta - myPose.theta;
			dt = fixangles(dt);
			ctrl.angle = k.a * dt;
			return true;
		}
	}
	ctrl.speed = k.p - k.iiris * fabsf(alpha);// * rho;
	ctrl.angle = k.a * alpha;

	cout<< "\tdx = " << dx << endl;
	cout<< "\tdy = " << dy << endl;
	cout<< "\tmy theta = " << myPose.theta << " " << r2d(myPose.theta) << endl;
	cout<< "\tatan2 dydx = " << ata << " " << r2d(ata) << endl;
	cout<< "\trho = " << rho << endl;
	cout<< "\talpha = " << alpha << " " << r2d(alpha) << endl;
	cout<< "\tv = " << ctrl.speed << endl;
	cout<< "\tw = " << ctrl.angle << " " << r2d(ctrl.angle) << endl;
	history.push_back(HistPoint{lastPose, Ctrl{ctrl.speed, ctrl.angle}, alpha, ata});
	if (rho < k.closeEnough) {
		cout << "\tCLOSE ENOUGH!" << endl;
		return nextMidpoint();
	}
	return true;
}


void MotionControl::drawInfo(SDL_Surface* screen, int sx, int sy) const {
	filledCircleRGBA(screen, sx, sy, 5, 255, 255, 255, 255);
	if (history.size()) {
		float a = history.back().ata;
		int dx = 20 * cos(a);
		int dy = 20 * sin(a);
		lineRGBA(screen, sx, sy, sx + dx, sy - dy, 255, 0, 0, 255);
	}
}
void MotionControl::drawMap(SDL_Surface* screen, int sx, int sy) const {
#if 0
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
#endif
}

// XXX this should maybe not be called from another thread while doing some parsing
// Currently only called once after starting, will probably be called after having
// finished the whole route
void MotionControl::setRoute(const LocList& route) {
	midpoints = route;
	dPrint(0, "reload waypoint from current location");
	routeStarting = true;
	startPoint.x = route.front().x;
	startPoint.y = route.front().y;
	auto it = route.begin(); ++it;
	float dx = it->x - startPoint.x;
	float dy = it->y - startPoint.y;
	startPoint.theta = atan2(dy, dx);
}

// Go to next midpoint and update midpoint list
// return false if no midpoint available
bool MotionControl::nextMidpoint(void) {
	midpoints.pop_front();
	if (midpoints.empty()) {
		// finished with current route -- cannot continue
		ctrl.speed = 0;
		ctrl.angle = 0;
		return false;
	}
	return true;
}

void MotionControl::stopBackingOff() {
	interface.iBehaviourCtrl->SetStart();
}

void MotionControl::backOff() {
	// ??!?
	interface.iBehaviourCtrl->SetStop();
	interface.iMotionCtrl->SetSpeed(-0.5, 0, -acceleration);
}

bool MotionControl::backFromGoal(SLAM::RobotLocation current) {
	const float farFromTarget = 0.5;
	if (sqrt(current.x * current.x + current.y * current.y) >= farFromTarget) {
		ctrl.speed = 0;
		ctrl.angle = 0;
		return false;
	}
	ctrl.speed = -0.1;
	ctrl.angle = 0;
	return true;
}

float MotionControl::routeLeft() const {
	if (midpoints.empty())
		return 0;
	float amount = 0;
	auto prev = midpoints.begin(), next = prev;
	for (++next; next != midpoints.end(); prev = next++) {
		int dx = abs(next->x - prev->x);
		int dy = abs(next->y - prev->y);
		if (dx + dy == 2) // diagonal walk
			amount += sqrt(2);
		else // 1
			amount++;
	}
	return amount / SLAM::MapData::unitSize;
}

const float MotionControl::acceleration = 0.3;

}
