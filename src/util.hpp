#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <sstream>
#include <string>
#include <cmath>

template <typename T>
std::string lexical_cast(T x) {
	std::stringstream s;
	s << x;
	return s.str();
}

static inline float r2d(float deg) { return deg / M_PI * 180; }

const float DIR_RIGHT = 0;
const float DIR_UP = M_PI / 2;
const float DIR_LEFT = M_PI;
const float DIR_DOWN = 3 * M_PI / 2;

// TODO: write angle_eq() that does modulus
static inline bool floateq(float a, float b, float eps = 0.001) {
	return fabsf(a - b) < eps;
}

#endif
