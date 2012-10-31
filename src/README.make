Building:
	make

The build system should know all about header dependencies, so if file
foo.cpp includes bar.h, then foo.cpp gets rebuilt if bar.h is modified.

The object and dependency files go to directories called obj and dep.
Those contain the same directory structure as the sources do.

Running:
	make execute_simulator
	or:
	make execute_real

Handling weird errors:
	tell sooda about them, there should be none
	if in doubt, try "make clean", or even "rm -rf out"
