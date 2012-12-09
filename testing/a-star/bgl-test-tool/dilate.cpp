#include "dilate.hpp"
#include "SDL.h"
#include "SDL_gfxPrimitives.h"
#include <iostream>

GridMap dilate(const GridMap& map, int amount) {
	const int w = map[0].size(), h = map.size();
	SDL_Surface* surf = SDL_CreateRGBSurface(0, w, h, 32, 0, 0, 0xff, 0);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			if (map[y][x]) {
				filledCircleColor(surf, x, y, amount, 0xffffff);
			}
		}
	}
	GridMap out(h, GridRow(w));
	for (int y = 0; y < h; y++) {
		Uint32* srcrow = reinterpret_cast<Uint32*>(static_cast<char*>(surf->pixels) + surf->pitch * y);
		GridRow& dstrow = out[y];
		for (int x = 0; x < w; x++) {
			if (srcrow[x])
				dstrow[x] = true;
		}
	}
	return out;
}

