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

# only one level of source directories should be enough
SRCS := $(wildcard *.cpp) $(wildcard */*.cpp)
OBJS := $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRCS))
DEPS := $(patsubst $(OBJDIR)/%,$(DEPDIR)/%.d,$(OBJS))

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
	-DLINUX_OS -Wall

# automatic dependency generation
CFLAGS += -MMD -MP -MF $(subst $(OBJDIR)/,$(DEPDIR)/,$(@)).d

# read LDFLAGS_ from command line too
LDFLAGS := $(LDFLAGS_) $(LIBDIRS) \
	-lMaCI -lGIMI -lGIMutils \
	-lrt -lpthread -lcrypto -ljpeg -lpng12 -lssl \
	`xml2-config --libs` `sdl-config  --libs` -lSDL_gfx -lSDL_image

# magic for automatic objdir generation
.SECONDEXPANSION:

all: $(TARGET)
	echo $(DEPS)

# convenience commands
execute_real: $(TARGET)
	./taistellaas 130.233.120.178 40002

execute_simulator: $(TARGET)
	./taistellaas localhost $(shell grep -wo 4002. ../../application/execute_simulator_client) FSRSim.J2B2

.PHONY: all execute_real execute_simulator

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

.PHONY: clean

-include $(DEPS)
