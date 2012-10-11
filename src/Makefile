
include standard_defs.mk

# Define targets
TARGETS = J2B2-UI-example

# All rule compiles all per default
.PHONY: all
all:	$(TARGETS)

# Define objects
OBJECTS = J2B2-UI-example.opp J2B2Demo.opp motion/motioncontrol.opp

include SLAM/Makefile

.PHONY: test
test: ;@echo "$(foreach OBJ, $(OBJECTS), "$(OBJ)            ")"

J2B2-UI-example: GIMlibs $(OBJECTS)
	@$(LINK) -o $@ $(OBJECTS) $(LIBS) -lSDL -lSDL_gfx -lSDL_image -lpng
	@echo "      [LD] $@"

# GIMlib stuff
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

# Cleanup
.PHONY: clean
clean:
	@rm -f *.o *.opp *.dopp $(OBJECTS) $(TARGETS)
	@echo "     [LOCAL-CLEAN]"
