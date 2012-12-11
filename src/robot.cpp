#include "robot.hpp"
#include "util.hpp"
#include "owndebug.h"
#include <stdexcept>
#include <iostream>
#include <vector>
#include <string>

Robot::Robot(CJ2B2Client& j2b2)
		: CThread(NUM_THREADS), j2b2(j2b2),
		motionControl(j2b2),
		servoControl(j2b2),
		slam(SLAM::SLAM(SLAM::RobotLocation(0, 0, 0))),
		navigation(),
		camera(j2b2, servoControl),
		lastMeas(),
		statistics(),
		manual()
	{
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
	statistics.taskStartTime = 0;

	manual.enabled = true;
	
	// Initialize the state machine for high level planner
	taskState = START;
	// Initialze the number of targets
	numberOfPickUps = 0;

	// returns a joystick structure that we don't use for anything
	// don't bother cleaning up, will do that later if necessary
	SDL_JoystickOpen(0);

	// need initial empty data before doing anything
	navigation.refreshMap(slam.getCurrentMapData());

	// start other threads, return immediately to caller
	RunThread(THREAD_MAIN);
	RunThread(THREAD_SENSE);
	RunThread(THREAD_CONTROL);
	RunThread(THREAD_SLAM);
	RunThread(THREAD_CAMERA);
	RunThread(THREAD_USER);
}

Robot::~Robot() {
	stop();
	wait();
}

void Robot::navigate(void) {
	motionControl.setRoute(navigation.getRoute());
}

void Robot::explore() {
	// TODO: write code to search the largest unknown area
	// currently, in this state, someone should click the destination with a mouse
	std::cout << "PLANNER: Hey user, click somewhere to explore!" << std::endl;
}

void Robot::updateTargets() {
	std::cout << "PLANNER: Targets:" << std::endl;
	targets = slam.getCurrentMapData().getObjects(SLAM::MapData::TARGET);
	if (targets.empty()) {
		std::cout << "PLANNER: (none)" << std::endl;
	}
	for (auto it = targets.begin(); it != targets.end();) {
		std::cout << "PLANNER: Target at: " << it->x << " " << it->y << std::endl;
		if (!navigation.isFloor(*it)) {
			std::cout << "PLANNER: Not reachable yet" << std::endl;
			it = targets.erase(it);
		} else {
			++it;
		}
	}
}
void Robot::selectTarget() {
	if (targets.empty()) {
		return;
	}
	int minId = 0, i = 0;
	float minDist = navigation.routeLength(targets.front());
	for (auto it = targets.begin(); it != targets.end(); ++it, ++i) {
		float dist = navigation.routeLength(*it);
		if (dist < minDist) {
			minDist = dist;
			minId = i;
		}
	}
	std::cout << "PLANNER: NEAREST: " << targets[minId].x << " " << targets[minId].y << " at " << minDist << std::endl;
	currentTarget = targets[minId];
	targets.erase(targets.begin() + minId);
}
void Robot::navigateTarget() {
	navigation.solveTo(currentTarget);
	navigate();

}

void Robot::planAction(void) {
	ownTime_ms_delta_t hurryUp = 1000*60*13; // If it's time to abandon everything else and get current targets to the goal 
	updateTargets();
	if (!manual.enabled) {
		if(statistics.taskStartTime == 0)
			statistics.taskStartTime = ownTime_get_ms();
		SLAM::RobotLocation p = slam.getCurrentMapData().getRobotLocation();
		std::cout << "PLANNER: ROBOT LOCATION " << p.x << " " << p.y << " " << p.theta << std::endl;
		switch (taskState) {
			case START: // roll around to discover empty areas
				if (!motionControl.rollStart(p))
					taskState = EXPLORE;
				break;
			case EXPLORE: // seek unexplored areas until targets visible
				if (targets.size()) {
					if (motionControl.running())
						motionControl.stop();
					taskState = GO_TO_TARGET;
					selectTarget();
					navigateTarget();
				} else {
					if (!motionControl.iterate(p)) {
						explore();
					}
				}
				break;
			case GO_TO_TARGET: // Move near the nearest target to open the hatch
				if (motionControl.routeLeft() < HATCH_OPEN_DISTANCE) {
					servoControl.setHatch(false);
					camera.rotateNear();
					// TODO: stop for taking better images?
					navigateTarget();
					taskState = PICK_UP;
				}
				motionControl.iterate(p);
				break;
			case PICK_UP: // Move on to the target and close the hatch when finished
				if (!motionControl.iterate(p)) {
					servoControl.setHatch(true);
					camera.rotateFar();
					numberOfPickUps++;
					if(numberOfPickUps >= 10) {
						taskState = GO_RETURN_TO_GOAL;
					} else {
						if (targets.empty()) {
							taskState = EXPLORE;
						} else {
							selectTarget();
							navigateTarget();
							taskState = GO_TO_TARGET;
						}
					}
				}
				break;
			case GO_RETURN_TO_GOAL: // Try to find a route to the goal -- if cannot, explore more
				if (navigation.solveTo(SLAM::Location(0, 0))) {
					navigate();
					taskState = RETURN_TO_GOAL;
				} else {
					taskState = EXPLORE;
				}
				break;
			case RETURN_TO_GOAL: // Going to goal
				if (!motionControl.iterate(p)) {
					float clear = navigation.wallClearance(p);
					float walk = clear < GOALWALK ? clear : GOALWALK;
					navigation.solveTo(SLAM::Location(p.x + cos(p.theta) * walk, p.y + sin(p.theta) * walk));
					navigate();
					taskState = GOAL_WALKHAX;
				}
				break;
			case GOAL_WALKHAX: // in goal, drive a bit forwards to be able to drop targets correctly
				if (!motionControl.iterate(p)) {
					taskState = RELEASE_TARGETS;
					servoControl.setHatch(false);
				}
				break;
			case RELEASE_TARGETS: // eggs hatching, get back
				servoControl.setHatch(false);
				if (!motionControl.backFromGoal(p)) {
					taskState = END_STATE;
				}
				// TODO: examine your balls
				break;
			case END_STATE: // world domination succeeded
				manual.enabled = true;
				break;
			case BACK_OFF: // bumpers hit. exit to exploring when bumpers not hitting anymore
				std::cout << "PLANNER: DYNDYNDYY" << std::endl;
				motionControl.backOff();
				break;
			default: throw std::runtime_error("Bad task state number"); break;
		}
		if(ownTime_get_ms_since(statistics.taskStartTime) >= hurryUp
				&& taskState != GO_RETURN_TO_GOAL
				&& taskState != RETURN_TO_GOAL
				&& taskState != RELEASE_TARGETS
				&& taskState != END_STATE
				&& taskState != BACK_OFF)
			taskState = GO_RETURN_TO_GOAL;
	}
	std::cout << "PLANNER: CURRENT TASK: " << taskdescr[taskState] << std::endl;
}

void Robot::threadMain(void) {
	SLAM::RobotLocation slamloc;
	motionControl.setPose(slamloc);
	while (!IsRequestTermination()) {
		// TODO: do some main planner magic here
		taskLock.Lock();
		planAction();
		taskLock.Unlock();
		ownSleep_ms(100); // TODO: determine good value to synch with motor control loop
	}
}

void Robot::wait(void) {
	WaitThread(THREAD_MAIN);
	WaitThread(THREAD_SENSE);
	WaitThread(THREAD_CONTROL);
	WaitThread(THREAD_SLAM);
	WaitThread(THREAD_CAMERA);
	WaitThread(THREAD_USER);
}

void Robot::threadSense(void) {
	// FIXME: separate events could be in different threads as they all have a timeout setting.
	// It's maybe fine to poll them very fast (with a small timeout), though
	int positionSeq = -1, laserSeq = -1, bumperSeq = -1;
	
//	unsigned int cameraSeq = 0;
	while (!IsRequestTermination()) {
		// TODO: call motionControl.pollPosition(); or something here
		MaCI::Position::CPositionData pd;
		if (j2b2.iPositionOdometry->GetPositionEvent(pd, &positionSeq, 10)) {
			statistics.odometry++;
			const MaCI::Position::TPose2D* pose = pd.GetPose2D();
			if (pose) {
				lastMeas.pose.set(*pose);
				slam.updateOdometryData(SLAM::RobotLocation(pose->x, pose->y, pose->a));
				navigation.updateLocation(slam.getCurrentMapData().getGridLocation());
			} else {
				dPrint(1, "WTF, got position event with no pose");
			}

		}

		// Fetch image from robot using Camera module
		try {
			camera.updateCameraData(slam.getCurrentMapData().getRobotLocation());
			lastMeas.image.set(camera.getCameraImage());
			statistics.camera++;
		} catch ( ... ) {
			dPrint(1, "WTF, got image data with no data");
		}

/*		MaCI::Image::CImageData imgData;
		if (j2b2.iImageCameraFront->GetImageData(imgData, &cameraSeq)) {
			MaCI::Image::CImageContainer image;
			if (imgData.GetImage(image, NULL, true)) {
				lastMeas.image.set(image);
				statistics.camera++;
			} else {
				dPrint(1, "WTF, got image data with no data");
			}
		}
*/
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
			bool hit = false;
			for(EACH_IN_i(bumperDistance)) {
				if (i->distance >= 0.00) {
					dPrint(1,"Bumpers hit: %f meters @ %f rad", i->distance, i->angle);
					taskLock.Lock();
					taskState = BACK_OFF;
					taskLock.Unlock();
					hit = true;
				}
			}
			if (taskState == BACK_OFF && hit == false) {
				taskState = EXPLORE;
				motionControl.stopBackingOff();
			}
			// TODO: do something meaningful if we crash
			// tell main planner about it
		}
	}
}

void Robot::threadControl(void) {
	while (!IsRequestTermination()) {
		// TODO: automatic driving (manual still optional)
		if (manual.enabled)
			motionControl.setSpeed(manual.speed, manual.angle);
		else
			motionControl.refreshSpeed();
		ownSleep_ms(200);
	}
}

void Robot::threadSlam(void) {
	while (!IsRequestTermination(0)) {
		if (statistics.lidar > statistics.slam) {
			if (statistics.odometry != 0)
				if (slam.updateLaserData(lastMeas.lidar.get()))
					navigation.refreshMap(slam.getCurrentMapData());
			statistics.slam++;
		}
	}
}

void Robot::threadCamera(void) {
	while (!IsRequestTermination(0)) {
		camera.updateToSLAM(slam);
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
	MaCI::Position::TPose2D pose = lastMeas.pose.get();
	info.push_back("Odometry pose: " + lexical_cast(pose.x) + " " + lexical_cast(pose.y) + " " + lexical_cast(pose.a));
	SLAM::RobotLocation slamloc = slam.getCurrentMapData().getRobotLocation();
	info.push_back("Map pose: " + lexical_cast(slamloc.x) + " " + lexical_cast(slamloc.y) + " " + lexical_cast(slamloc.theta));
	slamloc.x += slamcalib.x;
	slamloc.y += slamcalib.y;
	info.push_back("Map pose corrected: " + lexical_cast(slamloc.x) + " " + lexical_cast(slamloc.y) + " " + lexical_cast(slamloc.theta));
	info.push_back("Ctrlr: v=" + lexical_cast(motionControl.getCtrl().speed) + ", w=" + lexical_cast(motionControl.getCtrl().angle));


	MaCI::Image::CImageContainer image = lastMeas.image.get();
	if (image.GetImageDataType() == MaCI::Image::KImageDataJPEG
			&& image.GetImageDataPtr() != NULL) {
		SDL_RWops *rw;
		// TODO: const?
		rw = SDL_RWFromConstMem(static_cast<const void*>(image.GetImageDataPtr()),
				image.GetImageDataSize());
		SDL_Surface *image = IMG_LoadJPG_RW(rw);
		SDL_FreeRW(rw);
		SDL_Rect rcDest = {static_cast<Sint16>(2 * (SLAM::MapData::gridSize + 10)), 0, 0, 0};
		SDL_BlitSurface(image, NULL, screen, &rcDest);
		SDL_FreeSurface(image);
	} else {
		info.push_back("No camera image available");
	}

	slam.drawMapData(screen, win_width, win_height);
	slam.drawLaserData(screen, win_width, win_height);

	if (manual.enabled) {
		info.push_back("Manual steering");
	} else {
		info.push_back("Automatic steering");
	}

	const int slamGridEnd = 2 * (SLAM::MapData::gridSize + 10);

	navigation.draw(screen, slamGridEnd, slamGridEnd);
	motionControl.drawInfo(screen, SLAM::MapData::gridSize, slamGridEnd + SLAM::MapData::gridSize / 2);
	//motionControl.drawMap(screen, slamGridEnd, win_height-1-10);

	int y = slamGridEnd;
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
			case SDL_MOUSEBUTTONDOWN:
				if (taskState == EXPLORE && event.button.button == SDL_BUTTON_RIGHT) {
					const int slamGridEnd = 2 * (SLAM::MapData::gridSize + 10);
					int x = event.button.x - slamGridEnd, y = event.button.y - slamGridEnd;
					if (y >= 0 && y < SLAM::MapData::gridSize) {
						y = SLAM::MapData::gridSize - 1 - y;
						if (x >= SLAM::MapData::gridSize)
							x -= SLAM::MapData::gridSize;
						if (x >= 0 && x < SLAM::MapData::gridSize) {
							std::cout << "PLANNER: Find route: " << x << " " << y << std::endl;
							if (navigation.solveTo(SLAM::GridPoint(x, y)))
								navigate();
						}
					}
				}
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
			case SDLK_SPACE:
				manual.enabled = true;
				break;
			case SDLK_RETURN:
				manual.enabled = false;
				break;
			case SDLK_n:
				navigation.refreshMap(slam.getCurrentMapData());
				break;
			case SDLK_r:
				taskLock.Lock();
				taskState = GO_RETURN_TO_GOAL;
				taskLock.Unlock();
				break;
			case SDLK_h:
				j2b2.iTextToSpeech->SendText("Hello world");
				break;
			case SDLK_1:
				servoControl.setHatch(true);
				break;
			case SDLK_0:
				servoControl.setHatch(false);
				break;
			case SDLK_PAGEUP:
				camera.rotateFar();
/*
				float tilt = servoControl.getPosition(Motion::ServoControl::KServoCameraPTUTilt) - 0.05;
				std::cout << "CAM TILT AT: " << tilt << std::endl;
				servoControl.setPosition(Motion::ServoControl::KServoCameraPTUTilt, tilt);
*/
				break;
			case SDLK_PAGEDOWN:
				camera.rotateNear();
/*				float tilt = servoControl.getPosition(Motion::ServoControl::KServoCameraPTUTilt) + 0.05;
				std::cout << "CAM TILT AT: " << tilt << std::endl;
				servoControl.setPosition(Motion::ServoControl::KServoCameraPTUTilt, tilt);
*/
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
		case THREAD_CAMERA: threadCamera(); break;
		case THREAD_USER: threadUser(); break;
		default: throw std::runtime_error("Bad thread number"); break;
	}
	return 0;
}
const char* Robot::taskdescr[NUM_TASK_STATES] = {
	"Start",
	"Explore unknown",
	"Go to pick target",
	"Pick target up",
	"Go to returning to goal",
	"Return to goal",
	"Goal walkhax",
	"Release targets",
	"End state",
	"Back off"
};
