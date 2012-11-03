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
#include "SDL_gfxPrimitives.h"
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

template <typename T>
std::string lexical_cast(T x) {
	std::stringstream s;
	s << x;
	return s.str();
}

void eventloop(void) {
	const int w = 800, h = 600;
	SDL_Surface* screen = SDL_SetVideoMode(w, h, 32, SDL_SWSURFACE | SDL_DOUBLEBUF);
	float joyX = 0, joyY = 0;
	float minX = 0, maxX = 0;
	float minY = 0, maxY = 0;
	for (;;) {
		SDL_Event ev;
		while (SDL_PollEvent(&ev)) {
			switch (ev.type) {
				case SDL_QUIT:
					return;
				case SDL_JOYAXISMOTION:
					cout << "Joy axis " << (int)ev.jaxis.axis << " " << ev.jaxis.value << endl;
					if (ev.jaxis.axis == 0)
						joyX = ev.jaxis.value / 32768.0;
					else if (ev.jaxis.axis == 1)
						joyY = ev.jaxis.value / 32768.0;
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
		minX = min(minX, joyX);
		maxX = max(maxX, joyX);
		minY = min(minY, joyY);
		maxY = max(maxY, joyY);
		SDL_FillRect(screen, NULL, 0);
		filledCircleRGBA(screen, (int)(w / 2 * (1 + joyX)), (int)(h / 2 * (1 + joyY)), 20, 255, 255, 255, 255);
		string info = "minX " + lexical_cast(minX) + " maxX " + lexical_cast(maxX)
				+ " minY " + lexical_cast(minY) + " maxY " + lexical_cast(maxY);
		stringRGBA(screen, 0, 0, info.c_str(), 255, 255, 255, 255);
		SDL_Flip(screen);
		SDL_Delay(10);
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
