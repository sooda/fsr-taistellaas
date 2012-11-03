all: rungfx

rungfx: joygfx
	./joygfx

joygfx: joygfx.cpp
	g++ joygfx.cpp -o joygfx `sdl-config --libs --cflags` -lSDL_gfx

joy: joy.cpp
	g++ joy.cpp -o joy `sdl-config --libs --cflags`

.PHONY: run all
