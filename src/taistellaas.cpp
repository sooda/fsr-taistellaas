#include "J2B2-API.hpp"
#include "robot.hpp"
#include "owndebug.h"
#include <iostream>

int main(int argc, char *argv[]) {
	using std::cerr;
	using std::endl;

	if (argc < 3) {
		cerr << "Usage:" << endl
			<< argv[0] << " <GIMnetAP address> <GIMnetAP port> [Machine group]" << endl;
		return 1;
	}

	debugInit();
	debugSetGlobalDebugLvl(1);

	CJ2B2Client j2b2;

	if (!j2b2.Open(argv[1], atoi(argv[2]), "", argc > 3 ? argv[3] : "J2B2")) {
		cerr << "J2B2 Open() failed!" << endl;
		return 1;
	}

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) == -1) {
		cerr << "Failed to initialize SDL Video: " << SDL_GetError() << endl;
		return 1;
	}

	Robot robot(j2b2);
	robot.wait();

	SDL_Quit();

	return 0;
}
