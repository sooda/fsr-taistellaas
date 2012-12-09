#ifndef SEARCH_HPP
#define SEARCH_HPP

#include "gridutil.hpp"

std::pair<std::list<gridvertex>, my_pred_map> gridsearch(std::vector<std::vector<bool>>& grid, gridvertex goal);

#endif
