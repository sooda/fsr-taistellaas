// general interface for users, grid vertices and such basic stuff
#ifndef NAVI_GRIDUTIL_HPP
#define NAVI_GRIDUTIL_HPP
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

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
			return os << "(" << u.x << ", " << u.y << ")";
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
	const GridData& getData() const { return data; }
private:
	GridData& data;
};


#endif
