#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "thread.hpp"
#include "sync.hpp"
#include "J2B2-API.hpp"
#include "SLAM/includes.hpp"
#include "motion/motioncontrol.hpp"
#include "motion/ServoControl.hpp"
#include "cam/Camera.hpp"
#include "navi/navigation.hpp"
#include "motion/ServoControl.hpp"

// a measurement with a lock for safely transferring data between threads
template <class T>
class Measurement : private gim::CSync {
public:
	void set(const T& val) {
		Lock lock(*this);
		value = val;
	}
	// return a new value that can be safely manipulated later
	T get() {
		Lock lock(*this);
		return value;
	}
private:
	// RAII hack for ensuring efficient locking the class inside a function
	class Lock {
	public:
		Lock(gim::CSync& cs) : cs(cs) {
			cs.Lock();
		}
		~Lock() {
			cs.Unlock();
		}
	private:
		Lock(const Lock&);
		Lock& operator=(const Lock&);

		gim::CSync& cs;
	};

	T value;
};

class Robot : private gim::CThread {
public:
	Robot(CJ2B2Client &aClient);
	void stop(void);
	void wait(void);
	~Robot();

private:
	enum ThreadId {
		THREAD_MAIN,
		THREAD_SENSE,
		THREAD_CONTROL,
		THREAD_SLAM,
		THREAD_CAMERA,
		THREAD_USER,

		NUM_THREADS
	};
	
	enum TaskState {
		START,
		EXPLORE,
		GO_TO_TARGET,
		PICK_UP,
		GO_RETURN_TO_GOAL,
		RETURN_TO_GOAL,
		RELEASE_TARGETS,
		END_STATE,
		BACK_OFF,
		
		NUM_TASK_STATES
	};
	static const char* taskdescr[];

	int ThreadFunction(const int num);

	void threadMain(void);
	void threadSense(void);
	void threadControl(void);
	void threadSlam(void);
	void threadCamera(void);
	void threadUser(void);

	void navigate(void);
	void explore(void);
	void planAction(void);
	void updateTargets(void);
	void selectTarget(void);
	void navigateTarget(void);

	void pollEvents(void);
	void handleKey(int, SDLKey);
	void drawScreen(SDL_Surface*, int);

	CJ2B2Client& j2b2;
	Motion::MotionControl motionControl;
	Motion::ServoControl servoControl;
	SLAM::SLAM slam;
	Navi::Navigation navigation;
	cam::Camera camera;

	struct Measurements {
		Measurement<MaCI::Image::CImageContainer> image;
		Measurement<MaCI::Ranging::TDistanceArray> lidar;
		Measurement<MaCI::Position::TPose2D> pose;
	} lastMeas;

	struct Stats { // locking? let's assume these are atomically accessable. XXX should these members in measurement class?
		int odometry;
		int lidar;
		int camera;
		int slam;
		ownTime_ms_t startTime;
		ownTime_ms_t taskStartTime; // time when the automatic task is started (in final demo 15min is calculated from this moment)
	} statistics;

	struct {
		float x, y;
	} slamcalib;

	struct {
		float speed, angle;
		bool enabled;
	} manual;
	
	TaskState taskState;
	gim::CSync taskLock;
	int numberOfPickUps;
	std::vector<SLAM::Location> targets;
	SLAM::Location currentTarget;

	static const int win_width = 310 + 310 + 640, win_height = 310 + 310 + 310;
};

// open the hatch when distance from robot center to goal is < this
#define HATCH_OPEN_DISTANCE (0.24 + 0.5)

#endif
