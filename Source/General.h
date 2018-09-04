#pragma once

#define _SILENCE_CXX17_ADAPTOR_TYPEDEFS_DEPRECATION_WARNING
#define _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING

#define EIGEN_NO_DEBUG
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_PARALLELIZE
#define EIGEN_MPL2_ONLY

#define _USE_MATH_DEFINES

#include <vector>
#include <optional>
#include <iterator>
#include <math.h>
#include <memory>
#include <random>
#include <functional>
#include <algorithm>
#include <stack>
#include <cassert>


template<class T> using spvector = std::vector<std::shared_ptr<T>>;

static const float PI = (float)M_PI;
static const float ToDeg = 180.0f / (float)PI;
static const float ToRad = (float)PI / 180.0f;

inline int min( int x, int y ) { return x<y ? x : y; }
inline int max( int x, int y ) { return x > y ? x : y; }

inline float min(float x, float y) { return x<y ? x : y; }
inline float max(float x, float y) { return x > y ? x : y; }

inline int clamp( int x, int minVal, int maxVal ) { return max( minVal, min( maxVal, x ) ); }
inline float clamp(float x, float minVal, float maxVal) { return max(minVal, min(maxVal, x)); }
inline float clamp01(float x) { return clamp(x, 0.0f, 1.0f); }
inline float clampPositive(float x) { return max(x, 0.0f); }

inline bool inRange(float x, float minVal, float maxVal) { return minVal <= x && x <= maxVal; }
inline bool inRange01(float x) { return inRange(x, 0.0f, 1.0f); }

static std::random_device seed_gen;
static std::mt19937 engine(seed_gen());
static std::uniform_real_distribution<float> dist(0.0, 1.0);
inline float randf() {
	return dist(engine);
}
inline float randf( float max ) {
	return randf() * max;
}
inline float randf(float min, float max) {
	return randf(max - min) + min;
}
inline int randi(int n) {
	auto dist = std::uniform_int_distribution<int>(0, n - 1);
	return dist( engine );
}
template <class T> const T& randSelect(const std::vector<T>& v) {
	return v[randi(v.size())];
}


#include "Vector.h"
#include "Quaternion.h"