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
		THREAD_USER,

		NUM_THREADS
	};

	int ThreadFunction(const int num);

	void threadMain(void);
	void threadSense(void);
	void threadControl(void);
	void threadSlam(void);
	void threadUser(void);

	void navigate(void);

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
	} statistics;

	struct {
		float x, y;
	} slamcalib;

	struct {
		float speed, angle;
		bool enabled;
	} manual;

	static const int win_width = 1024, win_height = 768;
};

#endif
