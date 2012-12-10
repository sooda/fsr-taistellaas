#ifndef NAVI_GET_TURNS_HPP
#define NAVI_GET_TURNS_HPP

#include "gridutil.hpp"
#include <vector>
#include <utility>

/*
 * Simplify a continuous path that contains adjacent vertices.
 * Input: continuous path (abs(max(dx, dy)) == 1)
 * Output: points where the direction changes, next "walking direction" included (
 * .first = location, .second = next walk delta). Delta = (0, 0) for end point
 * Start point is not included if path contains only one point
 */
template <class VertIter>
std::vector<std::pair<vertex, vertex>> get_turns(VertIter start, VertIter end) {
	std::vector<std::pair<vertex, vertex>> turns;
	if (start == end) // no points
		return turns;

	VertIter prev2 = start; // store this and direction when reaching next node
	VertIter prev = start, current = ++start;

	if (current == end) {
		turns.push_back(std::make_pair(*prev2, vertex(0, 0)));
	} else {
		// delta from prev-1 to prev
		int prevdx = current->x - prev->x, prevdy = current->y - prev->y;
		int dx, dy;
		for (; current != end; prev = current++) {
			// delta from prev to current
			dx = current->x - prev->x;
			dy = current->y - prev->y;
			if (dx != prevdx || dy != prevdy) {
				// store prev-1 to prev; current dir not known yet
				turns.push_back(std::make_pair(*prev2, vertex(prevdx, prevdy)));
				prevdx = dx;
				prevdy = dy;
				prev2 = prev;
			}
		}
		turns.push_back(std::make_pair(*prev2, vertex(dx, dy)));
		turns.push_back(std::make_pair(*prev, vertex(0, 0)));
	}
	return turns;
}
#endif
