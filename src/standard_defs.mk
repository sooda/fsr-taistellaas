# Includes
INCLUDES := -I../../include \
-I../../include/ImageUtils \
-I../../include/ImageUtils/SVGImage \
-I../../include/ImageUtils/conversions \
-I../../include/ImageUtils/pnglite \
-I../../include/ImageUtils/PNGDecoder \
-I../../include/ImageUtils/JPEGEncoder \
-I../../include/ImageUtils/OpenCVStereoCalibrator \
-I../../include/ImageUtils/AC3DtoGL \
-I../../include/ImageUtils/JPEGDecoder \
-I../../include/ImageUtils/PNGEncoder \
-I../../include/ImageUtils/DistanceTransform \
-I../../include/ImageUtils/OpenGLTextureLoader \
-I../../include/utils \
-I../../include/MaCI \
-I../../include/MaCI/interfaces \
-I../../include/MaCI/interfaces/MaCICtrl \
-I../../include/MaCI/interfaces/MachineCtrl \
-I../../include/MaCI/interfaces/Position \
-I../../include/MaCI/interfaces/Dummy \
-I../../include/MaCI/interfaces/Ranging \
-I../../include/MaCI/interfaces/Unit \
-I../../include/MaCI/interfaces/SQL \
-I../../include/MaCI/interfaces/ObjectDB \
-I../../include/MaCI/interfaces/EnvironmentMeasurement \
-I../../include/MaCI/interfaces/JointGroupCtrl \
-I../../include/MaCI/interfaces/Audio \
-I../../include/MaCI/interfaces/Alarm \
-I../../include/MaCI/interfaces/common \
-I../../include/MaCI/interfaces/Recharger \
-I../../include/MaCI/interfaces/Map \
-I../../include/MaCI/interfaces/Energy \
-I../../include/MaCI/interfaces/TaskCtrl \
-I../../include/MaCI/interfaces/IO \
-I../../include/MaCI/interfaces/SpeedCtrl \
-I../../include/MaCI/interfaces/Image \
-I../../include/MaCI/interfaces/Gimbo \
-I../../include/MaCI/interfaces/Bearing \
-I../../include/MaCI/interfaces/Parameter \
-I../../include/MaCI/interfaces/IMU \
-I../../include/MaCI/interfaces/Wireless \
-I../../include/MaCI/interfaces/CoordinateDrive \
-I../../include/MaCI/interfaces/EmergencyStop \
-I../../include/MaCI/interfaces/Behaviour \
-I../../include/MaCI/interfaces/Text \
-I../../include/GIMI \
-I../../include/GIMnetAP \
-I../../include/GIMnetAP/protocol \

LDFLAGS := -L../../lib

# params
CC=gcc
CXX=g++
EXTRADEFS=-DLINUX_OS -DLINUX -fPIC $(CE)
WFLAGSC=-Wall -Wunused -Wshadow
WFLAGSCPP=-Wall -Wl,-t #-Wunused -Wshadow -Weffc++ #haha no warnings lol -mulppi
WFLAGSDCPP=

# Flags for DEVELOPEMENT phase
CFLAGS=-O0 -g3 $(WFLAGSC) $(INCLUDES) $(EXTRADEFS)
CPPFLAGS=-O0 -g3 $(WFLAGSCPP) $(INCLUDES) $(EXTRADEFS) `xml2-config --cflags`
DCPPFLAGS=-O0 -g3 $(WFLAGSDCPP) $(INCLUDES) $(EXTRADEFS)

LD=g++
LIBS=  -lMaCI -lGIMI -lGIMutils -lrt -lpthread -lssl -lcrypto -lSDL_gfx -lSDL_image \
	-ljpeg -lpng12 -lssl \
	`xml2-config --libs` `sdl-config  --libs` -lMaCI -lGIMI

# Define COMPILE and LINK commands
COMPILEC=$(CC) $(CFLAGS) -c
COMPILECPP=$(CXX) $(CPPFLAGS) -c
COMPILEDCPP=$(CXX) $(DCPPFLAGS) -c
LINK=$(LD) $(LDFLAGS) 

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

