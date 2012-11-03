all: run

run: joy
	./joy

joy: joy.cpp
	g++ joy.cpp -o joy `sdl-config --libs --cflags`

.PHONY: run all
