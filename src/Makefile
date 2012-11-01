# Note that you should not need to change anything in this file
# if you make modifications to the sources, or even add new sources!
#
# (except when the include or library requirements change)

# the output binary name
TARGET := taistellaas

# names of directories for object and dependency files.
# no source file name should contain these for this to work
OBJDIR := out/obj
DEPDIR := out/dep

# a couple of levels of source directories should be enough
SRCS := $(wildcard *.cpp) $(wildcard */*.cpp) \
	$(wildcard */*/*.cpp)
OBJS := $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRCS))
DEPS := $(patsubst $(OBJDIR)/%,$(DEPDIR)/%.d,$(OBJS))

# where external tools are installed
EXT_LIBS := ../../../ext-libs

# we could try clang some day, so keep these in variables
CXX := g++
LD := g++

# ugly list of include paths separeted in a different file
include Makefile.ext

# no ":=" here, automatic variables used in deps
# append CFLAGS_ from command line, if given
CFLAGS = $(CFLAGS_) -O0 -g3 \
	$(INCLUDES) \
	`xml2-config --cflags` \
	-DLINUX_OS -DLINUX -Wall

# automatic dependency generation
CFLAGS += -MMD -MP -MF $(subst $(OBJDIR)/,$(DEPDIR)/,$(@)).d

# read LDFLAGS_ from command line too
LDFLAGS := $(LDFLAGS_) $(LIBDIRS) \
	-lMaCI -lGIMI -lGIMutils \
	-lrt -lpthread -lcrypto -ljpeg -lpng12 -lssl \
	`xml2-config --libs` `sdl-config  --libs` -lSDL_gfx -lSDL_image \
	-lgridfastslam -lscanmatcher -llog -lsensor_range -lsensor_odometry -lsensor_base -lutils -lconfigfile \
	-L$(EXT_LIBS)/lib -Wl,-rpath=$(EXT_LIBS)/lib

# magic for automatic objdir generation
.SECONDEXPANSION:

all: $(TARGET)

# convenience commands
run_real: $(TARGET)
	./taistellaas 130.233.120.178 40002

simulate: $(TARGET)
	./taistellaas localhost $(shell grep -wo 4002. ../../application/execute_simulator_client) FSRSim.J2B2

simulate-gdb: $(TARGET)
	gdb -ex run --args ./taistellaas localhost $(shell grep -wo 4002. ../../application/execute_simulator_client) FSRSim.J2B2

simulate-valgrind: $(TARGET)
	valgrind ./taistellaas localhost $(shell grep -wo 4002. ../../application/execute_simulator_client) FSRSim.J2B2

.PHONY: all run_real simulate simulate-gdb simulate-valgrind

$(TARGET): $(OBJS)
	$(LD) $^ -o $@ $(LDFLAGS)

# build objects, require directories for objs and deps
$(OBJDIR)/%.o: %.cpp | $$(subst $(OBJDIR),$(DEPDIR),$$(@D))/.dir $$(@D)/.dir
	$(CXX) $(CFLAGS) -c $< -o $@

# a sentinel file to help generating identical directory structure for objs and deps
%/.dir:
	mkdir -p $(@D)
	touch $@

.PRECIOUS: %/.dir

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS)

help:
	@echo "Build targets:"
	@echo "    all: build only (run simply 'make')"
	@echo "    simulate: run the simulator client"
	@echo "    simulate-gdb: run the simulator client under the gnu debugger"
	@echo "    simulate-valgrind: run the simulator client under valgrind"
	@echo "    run-real: control the real robot"
	@echo "    clean: remove generated files"


.PHONY: clean help

-include $(DEPS)
