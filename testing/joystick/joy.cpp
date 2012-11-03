/*
 * Lab stick info:
 * axis 0: X
 * axis 1: Y
 * axis 2: ??? (doesn't seem to be twist axis)
 * axis 3: percentage slider
 * buttons: 7
 * hat: four directional buttons
 */
#include "SDL.h"
#include <iostream>

using namespace std;

void eventloop(void) {
	for (;;) {
		SDL_Event ev;
		while (SDL_PollEvent(&ev)) {
			switch (ev.type) {
				case SDL_QUIT:
					return;
				case SDL_JOYAXISMOTION:
					cout << "Joy axis " << (int)ev.jaxis.axis << " " << ev.jaxis.value << endl;
					break;
				case SDL_JOYHATMOTION:
					cout << "Joy hat " << (int)ev.jhat.value << endl;
					break;
				case SDL_JOYBUTTONDOWN:
					cout << "Joy button " << (int)ev.jbutton.button << endl;
				default:
					break;
			}
		}
	}
}

int main() {
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) == -1) {
		cerr << "Failed to init SDL: " << SDL_GetError() << endl;
		return 1;
	}
	cout << SDL_NumJoysticks() << endl;
	if (SDL_NumJoysticks() == 0)
		return 0;
	SDL_Joystick* stick = SDL_JoystickOpen(0);
	cout << SDL_JoystickNumAxes(stick) << endl;
	cout << SDL_JoystickNumButtons(stick) << endl;
	cout << SDL_JoystickNumBalls(stick) << endl;
	cout << SDL_JoystickNumHats(stick) << endl;
	eventloop();
	SDL_JoystickClose(stick);
	SDL_Quit();
}
