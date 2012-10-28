all: run

rundebug: a.out
	LD_LIBRARY_PATH=~/SFML-1.6/lib ./a.out

run: a.out
	LD_LIBRARY_PATH=~/SFML-1.6/lib ./a.out > /dev/null

a.out: code.cpp
	g++ -g -I ~/SFML-1.6/include code.cpp -lsfml-graphics -lsfml-window -lsfml-system -L ~/SFML-1.6/lib/ -std=c++0x
