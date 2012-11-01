Building:
	make

Building in parallel:
	make -j JOBS

	where JOBS is the number of simultaneous commands. A good value is the
	number of cores that your computer has. On a quadcore system, a 'make -j4'
	is approximately four times faster than 'make'.

The build system should know all about header dependencies, so if file
foo.cpp includes bar.h, then foo.cpp gets rebuilt if bar.h is modified.

The object and dependency files go to directories called obj and dep.
Those contain the same directory structure as the sources do.

Running:
	Do as instructed by 'make help'.

Handling weird errors:
	tell sooda about them, there should be none
	if in doubt, try "make clean", or even "rm -rf out"
