// general interface for users, grid vertices and such basic stuff
#ifndef GRIDUTIL_HPP
#define GRIDUTIL_HPP
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <cmath>
#include <iterator>
#include "astar_search.hpp"
#include <utility> // need for pair?
#include <stdexcept>
#include <iostream>
#include <vector>
#include <list>
#include <map>

class Grid {
public:
	virtual bool free_at(int x, int y) const = 0;
	virtual int width() const = 0;
	virtual int height() const = 0;
};

class vertex {
	public:
		int x, y; // const/mutable
		vertex() : x(-1), y(-1) {}
		vertex(int x, int y) : x(x), y(y) {}

		friend std::ostream& operator<<(std::ostream& os, const vertex& u) {
			os << "(" << u.x << ", " << u.y << ")";
		}
		bool operator==(const vertex& other) const {
			return x == other.x && y == other.y;
		}
		bool operator!=(const vertex& other) const {
			return !(*this == other);
		}
};

class gridvertex : public vertex {
	public:
		gridvertex() : vertex(), grid(NULL) {}
		gridvertex(int x, int y, const Grid* grid) : vertex(x, y), grid(grid) {}

		// for std::map
		bool operator<(const gridvertex& other) const {
			return id() < other.id();
		}
	private:
		int id() const {
			return y * grid->width() + x;
		}
		const Grid* grid;
};

typedef std::pair<gridvertex, gridvertex> gridedge;

typedef std::map<gridvertex, gridvertex> my_pred_map;
class VectorGrid : public Grid {
public:
	typedef std::vector<std::vector<bool>> GridData;
	VectorGrid(GridData& data) : data(data) {}
	bool free_at(int x, int y) const {
		if (x < 0 || y < 0 || x >= width() || y >= height())
			return false;
		if (data[y][x])
			return false;
		return true;
	}
	int width() const { return data[0].size(); }
	int height() const { return data.size(); }
private:
	GridData& data;
};

#endif
