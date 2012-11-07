#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <sstream>
#include <string>

template <typename T>
std::string lexical_cast(T x) {
	std::stringstream s;
	s << x;
	return s.str();
}

#endif
