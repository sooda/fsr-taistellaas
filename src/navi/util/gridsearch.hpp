#ifndef NAVI_SEARCH_HPP
#define NAVI_SEARCH_HPP

#include "gridutil.hpp"
#include <list>

std::list<gridvertex> gridsearch(const VectorGrid& grid, gridvertex start, gridvertex goal);

#endif
