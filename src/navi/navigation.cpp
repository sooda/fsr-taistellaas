#include "navi/navigation.hpp"
#include "util.hpp"
#include "navi/util/gridsearch.hpp"

using SLAM::MapData;

namespace Navi {

Navigation::Navigation() :
		wallmap_orig(std::vector<std::vector<bool>>(
				MapData::gridSize, std::vector<bool>(MapData::gridSize, true)
		)),
		wallmap_dila(wallmap_orig)
		{
}
void Navigation::refreshMap(const SLAM::MapData& data) {
	map = data;
	for (int y = 0; y < MapData::gridSize; y++) {
		for (int x = 0; x < MapData::gridSize; x++) {
			double wall = map.getCellValue(SLAM::GridPoint(x,y), MapData::WALL);
			wallmap_orig[y][x] = !(wall >= -0.1 && wall <= 0.8);
		}
	}
	wallmap_dila = dilate(wallmap_orig, 5); // TODO: correct amount
}

void drawGrid(const GridMap& grid, SDL_Surface* screen, int x0, int y0) {
	const int gridsz = MapData::gridSize;
	for (int x = 0; x < gridsz; x++) {
		for (int y = 0; y < gridsz; y++) {
			int color = grid[y][x] ? 255 : 0;

			pixelRGBA(screen,
				x0 + x,
				y0 + gridsz-1 - y,
				color, color, color, 255);
		}
	}
}

bool Navigation::solveGrid(int x, int y) {
	SLAM::RobotLocation loc = map.getGridLocation();
	VectorGrid container(wallmap_dila);
	gridvertex start(loc.x, loc.y, &container);
	gridvertex goal(x, y, &container);
	current_route = gridsearch(container, start, goal);
	return !current_route.empty();
}

void Navigation::draw(SDL_Surface* screen, int posx, int posy) const {
//void SLAM::drawMapData(SDL_Surface* screen, const int window_width, const int window_height) {

	const int gridsz = MapData::gridSize;

	int x0 = posx;
	int y0 = posy;

	drawGrid(wallmap_orig, screen, x0, y0);
	drawGrid(wallmap_dila, screen, x0 + MapData::gridSize, y0);

	// draw the robot
	SLAM::RobotLocation loc = map.getGridLocation();
	filledCircleRGBA(screen, 
		x0 + loc.x, 
		y0 + gridsz-1 - loc.y, 
		(int)(0.4 / MapData::unitSize / 2), 0, 255, 255, 255);

	for (int i = 0; i < 10; i++) {
		pixelRGBA(screen,
			x0 + loc.x+i*cos(loc.theta),
			y0 + gridsz-1 - loc.y-i*sin(loc.theta),
			255, 0, 0, 255);
	}  
	pixelRGBA(screen, x0 + MapData::gridSize + loc.x, y0 + gridsz-1 - loc.y, 255, 0, 0, 255);

	for (auto it = current_route.begin(); it != current_route.end(); ++it) {
		pixelRGBA(screen, x0 + it->x, y0 + gridsz-1 - it->y, 0, 255, 0, 255);
	}
	for (auto it = current_route.begin(); it != current_route.end(); ++it) {
		pixelRGBA(screen, x0 + MapData::gridSize + it->x, y0 + gridsz-1 - it->y, 0, 255, 0, 255);
	}
}

Navigation::LocList Navigation::getRoute(void) const {
	Navigation::LocList out;
	for (auto it = current_route.begin(); it != current_route.end(); ++it)
		out.push_back(SLAM::MapData::grid2loc(SLAM::GridPoint(it->x, it->y)));
	return out;
}

}

