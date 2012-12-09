#ifndef NAVI_DILATE_HPP
#define NAVI_DILATE_HPP

#include <vector>

namespace Navi {

typedef std::vector<bool> GridRow;
typedef std::vector<GridRow> GridMap;

GridMap dilate(const GridMap& map, int amount);

}

#endif
