#ifndef DILATE_HPP
#define DILATE_HPP

#include <vector>

typedef std::vector<bool> GridRow;
typedef std::vector<GridRow> GridMap;

GridMap dilate(const GridMap& map, int amount);

#endif
