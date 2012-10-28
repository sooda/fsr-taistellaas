# GDIR define
include Makefile.ext

# params
CC=gcc
CXX=g++
EXTRADEFS=-DLINUX_OS $(CE)
WFLAGSC=-Wall -Wunused -Wshadow
WFLAGSCPP=-Wall -Wunused -Wshadow -Weffc++
WFLAGSDCPP=

# Flags for DEVELOPEMENT phase
CFLAGS=-O0 -g3 $(WFLAGSC) $(INCLUDES) $(EXTRADEFS)
CPPFLAGS=-O0 -g3 $(WFLAGSCPP) $(INCLUDES) $(EXTRADEFS) `xml2-config --cflags`
DCPPFLAGS=-O0 -g3 $(WFLAGSDCPP) $(INCLUDES) $(EXTRADEFS)

LD=g++
LIBS=  -lMaCI -lGIMI -lGIMutils -lrt -lpthread -lssl -lcrypto -lSDL_gfx -lSDL_image \
		-ljpeg -lpng12 -lssl `pkg-config opencv --cflags --libs` \
			`xml2-config --libs` `sdl-config  --libs` -lMaCI -lGIMI


# Define COMPILE and LINK commands
COMPILEC=$(CC) $(CFLAGS) -c
COMPILECPP=$(CXX) $(CPPFLAGS) -c
COMPILEDCPP=$(CXX) $(DCPPFLAGS) -c
LINK=$(LD) $(LDFLAGS)

.PHONY: all

# Define targets
TARGETS=J2B2-UI-example  
# All rule compiles all per default
all:	$(TARGETS)

###############################################
###############################################
###############################################
# J2B2-UI-example target, and dependencies
J2B2-UI-EXAMPLE_OBJECTS=J2B2-UI-example.opp J2B2Demo.opp SLAM/SLAM.opp SLAM/MapData.opp \
		motion/motioncontrol.opp cam/Camera.opp cam/Camutil.opp
J2B2-UI-example: GIMlibs $(J2B2-UI-EXAMPLE_OBJECTS)
	@$(LINK) -o $@ $(J2B2-UI-EXAMPLE_OBJECTS) $(LIBS) -lSDL -lSDL_gfx -lSDL_image -lpng
	@echo "      [LD] $@"
###############################################
###############################################
###############################################
PHONY: libGIMI.a libMaCI.a libGIMutils.a libSerialLinkClient.a 
libGIMI.a:
	+@if [ -d $(GDIR)/GIMI ]; then cd $(GDIR)/GIMI; make libGIMI.a-dist; fi
libMaCI.a:
	+@if [ -d $(GDIR)/MaCI ]; then cd $(GDIR)/MaCI; make libMaCI.a-dist; fi
libGIMutils.a:
	+@if [ -d $(GDIR)/utils ]; then cd $(GDIR)/utils; make libGIMutils.a-dist; fi
libSerialLinkClient.a:
	+@if [ -d $(GDIR)/GIMnetApps/SerialLink ]; then cd $(GDIR)/GIMnetApps/SerialLink; make CE="-O2" libSerialLinkClient.a; fi

.PHONY: GIMlibs
GIMlibs: libGIMI.a libMaCI.a libGIMutils.a libSerialLinkClient.a 
###############################################
.PHONY: clean
clean:
	@rm -f *.o *.opp *.dopp $(J2B2-UI-EXAMPLE_OBJECTS) $(TARGETS)
	@echo "     [LOCAL-CLEAN]"

########### Compile rules ####################
# Compile rule for all .o (.c) file
%.o:	%.c
	@$(COMPILEC) -o $@ $<
	@echo "      [CC] $<"

# Compile rule for all .opp (.cpp) files
%.opp:	%.cpp
	@$(COMPILECPP) -o $@ $<
	@echo "      [CXX] $<"

# Compile rule for all .dopp (dirty .cpp) files
%.dopp:	%.cpp
	@$(COMPILEDCPP) -o $@ $<
	@echo "      [DIRTY-CXX] $<"

##############################################
