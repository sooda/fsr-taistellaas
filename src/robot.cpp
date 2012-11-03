#include "robot.hpp"
#include "owndebug.h"
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>

Robot::Robot(CJ2B2Client& j2b2)
		: CThread(NUM_THREADS), j2b2(j2b2),
		motionControl(j2b2),
		slam(SLAM::SLAM(100, 100, 100, 100, SLAM::RobotLocation(0, 0, 0), MaCI::Ranging::TDistanceArray())),
		lastMeas(),
		statistics(),
		manual() {
	// sanity check: are the necessary services available?
	if (!j2b2.iPositionOdometry)
		throw std::runtime_error("Odometry not found");
	if (!j2b2.iImageCameraFront)
		throw std::runtime_error("Camera not found");
	if (!j2b2.iRangingBumpers)
		throw std::runtime_error("Bumpers not found");
	if (!j2b2.iRangingLaser)
		throw std::runtime_error("Laser not found");

	// for fps calculation
	statistics.startTime = ownTime_get_ms();

	// returns a joystick structure that we don't use for anything
	// don't bother cleaning up, will do that later if necessary
	SDL_JoystickOpen(0);

	// start other threads, return immediately to caller
	RunThread(THREAD_MAIN);
	RunThread(THREAD_SENSE);
	RunThread(THREAD_CONTROL);
	RunThread(THREAD_SLAM);
	RunThread(THREAD_USER);
}

Robot::~Robot() {
	stop();
	wait();
}

void Robot::threadMain(void) {
	while (!IsRequestTermination()) {
		// TODO: do some main planner magic here
		ownSleep_ms(100);
	}
}

void Robot::wait(void) {
	WaitThread(THREAD_MAIN);
	WaitThread(THREAD_SENSE);
	WaitThread(THREAD_CONTROL);
	WaitThread(THREAD_SLAM);
	WaitThread(THREAD_USER);
}

void Robot::threadSense(void) {
	// FIXME: separate events could be in different threads as they all have a timeout setting.
	// It's maybe fine to poll them very fast (with a small timeout), though
	int positionSeq = -1, laserSeq = -1, bumperSeq = -1;
	unsigned int cameraSeq = 0;
	while (!IsRequestTermination()) {
		// TODO: call motionControl.pollPosition(); or something here
		MaCI::Position::CPositionData pd;
		if (j2b2.iPositionOdometry->GetPositionEvent(pd, &positionSeq, 10)) {
			statistics.odometry++;
			const MaCI::Position::TPose2D* pose = pd.GetPose2D();
			if (pose)
				slam.updateOdometryData(SLAM::RobotLocation(pose->x, pose->y, pose->a));
			else
				dPrint(1, "WTF, got position event with no pose");
		}

		MaCI::Image::CImageData imgData;
		if (j2b2.iImageCameraFront->GetImageData(imgData, &cameraSeq)) {
			MaCI::Image::CImageContainer image;
			if (imgData.GetImage(image, NULL, true)) {
				lastMeas.image.set(image);
				statistics.camera++;
			} else {
				dPrint(1, "WTF, got image data with no data");
			}
		}

		// TODO: actually use the additional data for something
		// timestamp sounds useful
		MaCI::Ranging::TDistanceHeader laserHeader;
		MaCI::Common::TTimestamp laserTimestamp;
		MaCI::Ranging::TDistanceArray laserDistance;
		if (j2b2.iRangingLaser->GetDistanceArray(laserDistance, &laserHeader, &laserTimestamp, &laserSeq, 10)) {
			lastMeas.lidar.set(laserDistance);
			statistics.lidar++;
		}

		MaCI::Ranging::TDistanceHeader bumperHeader;
		MaCI::Common::TTimestamp bumperTimestamp;
		MaCI::Ranging::TDistanceArray bumperDistance;
		if (j2b2.iRangingBumpers->GetDistanceArray(bumperDistance, &bumperHeader, &bumperTimestamp, &bumperSeq, 10)) {
			// four bumpers, one has data, others at distance -1
			for(EACH_IN_i(bumperDistance)) {
				if (i->distance >= 0.00)
					dPrint(1,"Bumpers hit: %f meters @ %f rad", i->distance, i->angle);
			}
			// TODO: do something meaningful if we crash
			// tell main planner about it
		}
	}
}

void Robot::threadControl(void) {
	while (!IsRequestTermination()) {
		// TODO: automatic driving (manual still optional)
		motionControl.setSpeed(manual.speed, manual.angle);
		ownSleep_ms(200);
	}
}

void Robot::threadSlam(void) {
	while (!IsRequestTermination(0)) {
		if (statistics.lidar > statistics.slam) {
			if (statistics.odometry != 0)
				slam.updateLaserData(lastMeas.lidar.get());
			statistics.slam++;
		}
	}
}

void Robot::threadUser(void) {
	SDL_Surface *screen = SDL_SetVideoMode(win_width, win_height, 32, SDL_HWSURFACE | SDL_DOUBLEBUF);
	if (!screen) {
		dPrint(1, "Unable to set SDL video mode: %s\n", SDL_GetError());
		return;
	}

	SDL_WM_SetCaption("taistellaas", "taistellaas");

	int frames = 0;
	while (!IsRequestTermination()) {
		pollEvents();
		SDL_FillRect(screen, NULL, 0);
		drawScreen(screen, frames);
		SDL_Flip(screen);
		frames++;
		ownSleep_ms(10);
	}
}

template <typename T>
std::string lexical_cast(T x) {
	std::stringstream s;
	s << x;
	return s.str();
}

void Robot::drawScreen(SDL_Surface* screen, int frameno) {
	std::vector<std::string> info;
	ownTime_ms_delta_t timediff = ownTime_get_ms_since(statistics.startTime);
	float secs = timediff / 1000;
	info.push_back("Hello world");
	info.push_back("Odometry readings: "
			+ lexical_cast(statistics.odometry) + " " + lexical_cast(statistics.odometry / secs) + "/s");
	info.push_back("Lidar readings:    "
			+ lexical_cast(statistics.lidar) + " " + lexical_cast(statistics.lidar / secs) + "/s");

	info.push_back("Camera readings:   "
			+ lexical_cast(statistics.camera) + " " + lexical_cast(statistics.camera / secs) + "/s");

	info.push_back("Slam updates:      "
			+ lexical_cast(statistics.slam) + " " + lexical_cast(statistics.slam / secs) + "/s");

	info.push_back("Frames drawn:      "
			+ lexical_cast(frameno + 1) + " " + lexical_cast((frameno+1) / secs) + "/s");


	MaCI::Image::CImageContainer image = lastMeas.image.get();
	if (image.GetImageDataType() == MaCI::Image::KImageDataJPEG
			&& image.GetImageDataPtr() != NULL) {
		SDL_RWops *rw;
		// TODO: const?
		rw = SDL_RWFromConstMem(static_cast<const void*>(image.GetImageDataPtr()),
				image.GetImageDataSize());
		SDL_Surface *image = IMG_LoadJPG_RW(rw);
		SDL_FreeRW(rw);
		SDL_Rect rcDest = {0,0,0,0}; // or NULL?
		SDL_BlitSurface(image, NULL, screen, &rcDest);
		SDL_FreeSurface(image);
	} else {
		info.push_back("No camera image available");
	}

	slam.drawMapData(screen, win_width, win_height);
	slam.drawLaserData(screen, win_width, win_height);

	int y = 200;
	for (std::vector<std::string>::iterator it = info.begin(); it != info.end(); ++it) {
		stringRGBA(screen, 0, y, it->c_str(), 0, 255, 0, 255);
		y += 10;
	}
}

void Robot::stop(void) {
	SetRequestTermination(0);
}
void Robot::pollEvents(void) {
	SDL_Event event;
	static bool joystickMode = false;
	while (SDL_PollEvent(&event)) {
		switch (event.type) {
			case SDL_QUIT:
				stop();
				break;
			case SDL_KEYDOWN:
			case SDL_KEYUP:
				handleKey(event.type, event.key.keysym.sym);
				break;
			case SDL_JOYAXISMOTION:
				// control with a joystick if a joystick button is pressed simultaneously
				if (joystickMode) {
					if (event.jaxis.axis == 0)
						manual.angle = -event.jaxis.value / 32768.0 * M_PI;
					else if (event.jaxis.axis == 1)
						manual.speed = -event.jaxis.value / 32768.0 * 0.3;
				}
				break;
			case SDL_JOYBUTTONDOWN:
				joystickMode = true;
				break;
			case SDL_JOYBUTTONUP:
				joystickMode = false;
				manual.angle = 0;
				manual.speed = 0;
				break;
			default:
				break;
		}
	}
}
void Robot::handleKey(int type, SDLKey key) {
	if (type == SDL_KEYDOWN) {
		switch (key) {
			case SDLK_UP:
				manual.speed = 0.2;
				break;
			case SDLK_LEFT:
				manual.angle = M_PI / 10;
				break;
			case SDLK_DOWN:
				manual.speed = -0.2;
				break;
			case SDLK_RIGHT:
				manual.angle = -M_PI / 10;
				break;
			default:
				dPrint(1, "SDL KEYDOWN event: %d/%s",
						key,
						SDL_GetKeyName(key));
				break;
		}
	} else if (type == SDL_KEYUP) {
		switch (key) {
			case SDLK_UP:
			case SDLK_DOWN:
				manual.speed = 0.00;
				break;
			case SDLK_LEFT:
			case SDLK_RIGHT:
				manual.angle = 0.00;
				break;
			default:
				break;
		}
	} else {
		throw std::runtime_error("Unexpected key event type");
	}
}

int Robot::ThreadFunction(int threadId) {
	switch (threadId) {
		case THREAD_MAIN: threadMain(); break;
		case THREAD_SENSE: threadSense(); break;
		case THREAD_CONTROL: threadControl(); break;
		case THREAD_SLAM: threadSlam(); break;
		case THREAD_USER: threadUser(); break;
		default: throw std::runtime_error("Bad thread number"); break;
	}
	return 0;
}
