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
// clear the starting point so that we don't accidentally get stuck in the beginning
// also empty the current location so nearby walls don't affect us
GridMap startptshit(const GridMap& map, const SLAM::GridPoint& pt, int amount) {
	const int w = map[0].size(), h = map.size();
	SDL_Surface* surf = SDL_CreateRGBSurface(0, w, h, 32, 0, 0, 0xff, 0);
	SDL_FillRect(surf, NULL, 0);
	filledCircleColor(surf, pt.x, pt.y, amount, 0xffffff);
	GridMap out(map);
	for (int y = 0; y < h; y++) {
		Uint32* srcrow = reinterpret_cast<Uint32*>(static_cast<char*>(surf->pixels) + surf->pitch * y);
		GridRow& dstrow = out[y];
		for (int x = 0; x < w; x++) {
			if (srcrow[x])
				dstrow[x] = false;
		}
	}
	return out;
}

void Navigation::refreshMap(const SLAM::MapData& data) {
	map = data;
	GridMap obstacles(wallmap_orig);
	for (int y = 0; y < MapData::gridSize; y++) {
		for (int x = 0; x < MapData::gridSize; x++) {
			double wall = map.getCellValue(SLAM::GridPoint(x,y), MapData::WALL);
			wallmap_orig[y][x] = !(wall >= -0.1 && wall <= 0.25);

			double obst = map.getCellValue(SLAM::GridPoint(x,y), MapData::OBSTACLE);
			obstacles[y][x] = obst >= 0.25;
		}
	}
	const int sz = SLAM::MapData::gridSize;
	const int pienimulku = 2;
	obstacles = dilate(obstacles, round(0.15 / SLAM::MapData::unitSize));
	wallmap_dila = dilate(wallmap_orig, round(0.3 / SLAM::MapData::unitSize));
	wallmap_dila = startptshit(wallmap_dila, SLAM::GridPoint(sz/2, sz/2), pienimulku);
	auto bot = map.getRobotLocation();
	SLAM::Location botp(bot.x, bot.y);
	wallmap_dila = startptshit(wallmap_dila, SLAM::MapData::loc2grid(botp), pienimulku);
	for (int y = 0; y < MapData::gridSize; y++) {
		for (int x = 0; x < MapData::gridSize; x++) {
			wallmap_dila[y][x] = wallmap_dila[y][x] || obstacles[y][x];
		}
	}
}
void Navigation::refreshTargets(const std::vector<SLAM::Location>& tgts) {
	targets = tgts;
}
void Navigation::updateLocation(SLAM::RobotLocation pos) {
	roboPos = pos;
}

bool Navigation::isFloor(SLAM::Location loc) const {
	SLAM::GridPoint gp = SLAM::MapData::loc2grid(loc);
	return !wallmap_dila[gp.y][gp.x];
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

float Navigation::wallClearance(SLAM::RobotLocation source) const {
	float walkx = cos(source.theta);
	float walky = sin(source.theta);
	SLAM::GridPoint pt = SLAM::MapData::loc2grid(SLAM::Location(source.x, source.y));
	float x = pt.x;
	float y = pt.y;
	while (!wallmap_dila[int(y)][int(x)]) {
		x += walkx;
		y += walky;
	}
	float dx = x - pt.x;
	float dy = y - pt.y;
	return sqrt(dx * dx + dy * dy) * SLAM::MapData::unitSize;
}

search_info Navigation::findRoute(SLAM::GridPoint point) {
	VectorGrid container(wallmap_dila);
	gridvertex start(roboPos.x, roboPos.y, &container);
	gridvertex goal(point.x, point.y, &container);
	return gridsearch(container, start, goal);
}

float Navigation::routeLength(SLAM::Location loc) {
	return findRoute(SLAM::MapData::loc2grid(loc)).second / SLAM::MapData::unitSize;
}

bool Navigation::navigable(SLAM::Location loc) {
	return !findRoute(SLAM::MapData::loc2grid(loc)).first.empty();
}
SLAM::Location Navigation::farthest() {
	VectorGrid container(wallmap_dila);
	gridvertex start(roboPos.x, roboPos.y, &container);
	gridvertex goal(0, 0, &container); // usually wall
	gridvertex place_to_be = gridsearch_farthest(container, start, goal);
	return SLAM::MapData::grid2loc(SLAM::GridPoint(place_to_be.x, place_to_be.y));
}
bool Navigation::solveTo(SLAM::GridPoint point) {
	current_route = findRoute(point).first;
	return !current_route.empty();
}

bool Navigation::solveTo(SLAM::Location loc) {
	return solveTo(SLAM::MapData::loc2grid(loc));
}

void Navigation::draw(SDL_Surface* screen, int posx, int posy) const {
//void SLAM::drawMapData(SDL_Surface* screen, const int window_width, const int window_height) {

	const int gridsz = MapData::gridSize;

	int x0 = posx;
	int y0 = posy;

	drawGrid(wallmap_orig, screen, x0, y0);
	drawGrid(wallmap_dila, screen, x0 + MapData::gridSize, y0);

	// draw the robot
	filledCircleRGBA(screen, 
		x0 + roboPos.x, 
		y0 + gridsz-1 - roboPos.y, 
		(int)(0.4 / MapData::unitSize / 2), 0, 255, 255, 255);

	for (int i = 0; i < 10; i++) {
		pixelRGBA(screen,
			x0 + roboPos.x+i*cos(roboPos.theta),
			y0 + gridsz-1 - roboPos.y-i*sin(roboPos.theta),
			255, 0, 0, 255);
	}  
	pixelRGBA(screen, x0 + MapData::gridSize + roboPos.x, y0 + gridsz-1 - roboPos.y, 255, 0, 0, 255);

	for (auto it = current_route.begin(); it != current_route.end(); ++it) {
		pixelRGBA(screen, x0 + it->x, y0 + gridsz-1 - it->y, 0, 255, 0, 255);
	}
	for (auto it = current_route.begin(); it != current_route.end(); ++it) {
		pixelRGBA(screen, x0 + MapData::gridSize + it->x, y0 + gridsz-1 - it->y, 0, 255, 0, 255);
	}
	for (auto it = targets.begin(); it != targets.end(); ++it) {
		SLAM::GridPoint pt(SLAM::MapData::loc2grid(*it));
		pixelRGBA(screen, x0 + MapData::gridSize + pt.x, y0 + gridsz-1 - pt.y, 200, 0, 200, 255);
	}
}

Navigation::LocList Navigation::getRoute(void) const {
	Navigation::LocList out;
	for (auto it = current_route.begin(); it != current_route.end(); ++it)
		out.push_back(SLAM::MapData::grid2loc(SLAM::GridPoint(it->x, it->y)));
	return out;
}

}

