SRC := sfmldemo.cpp gridsearch.cpp
HDR := gridsearch.hpp gridutil.hpp gridgraph.hpp

all: run

rundebug: a.out
	LD_LIBRARY_PATH=~/SFML-1.6/lib ./a.out | tee log

run: a.out
	LD_LIBRARY_PATH=~/SFML-1.6/lib ./a.out > /dev/null

a.out: $(SRC) $(HDR)
	g++ -g -I ~/SFML-1.6/include $(SRC) -lsfml-graphics -lsfml-window -lsfml-system -L ~/SFML-1.6/lib/ -std=c++0x
