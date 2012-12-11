#ifndef NAVI_SEARCH_HPP
#define NAVI_SEARCH_HPP

#include "gridutil.hpp"
#include <list>

typedef std::pair<std::list<gridvertex>, float> search_info;
search_info gridsearch(const VectorGrid& grid, gridvertex start, gridvertex goal);

#endif
