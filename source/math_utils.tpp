#pragma once

#include <cmath>

#include "math_utils.h"
#include "common_types.h"

// Simple math utilities

namespace indoor_context {
// This template provides a compile-time mapping from two input type to
// the appropriate output type
template <typename T, typename G>
struct min_max_result {
	typedef T type;
};

// If the inputs are identical then the output is the same:
template <typename T> struct min_max_result<T, T> { typedef T type; };

// Otherwise use the more precise of the two types, and use unsigned
// results only if both arguments are unsigned
template <> struct min_max_result<int, unsigned int> { typedef int type; };
template <> struct min_max_result<unsigned int, int> { typedef int type; };
template <> struct min_max_result<int, float> { typedef float type; };
template <> struct min_max_result<unsigned int, float> { typedef float type; };
template <> struct min_max_result<float, int> { typedef float type; };
template <> struct min_max_result<float, unsigned int> { typedef float type; };
template <> struct min_max_result<int, double> { typedef double type; };
template <> struct min_max_result<unsigned int, double> { typedef double type; };
template <> struct min_max_result<double, int> { typedef double type; };
template <> struct min_max_result<double, unsigned int> { typedef double type; };
template <> struct min_max_result<float, double> { typedef float type; };
template <> struct min_max_result<double, float> { typedef float type; };

// Get minimum of two values
template <typename T, typename G>
inline typename min_max_result<T, G>::type
min(const T a, const G b) {
	typedef typename min_max_result<T, G>::type R;
	return static_cast<R>(a) < static_cast<R>(b) ? a : b;
}

// Get maximum of two values
template <typename T, typename G>
inline typename min_max_result<T, G>::type
max(const T a, const G b) {
	typedef typename min_max_result<T, G>::type R;
	return static_cast<R>(a) > static_cast<R>(b) ? a : b;
}

// Round and cast to integer
template <typename T>
inline int roundi(const T x) {
	return static_cast<int>(round(x));
}

// Ceil and cast to integer
template <typename T>
inline int ceili(const T x) {
	return static_cast<int>(ceil(x));
}

// Floor and cast to integer
template <typename T>
inline int floori(const T x) {
	return static_cast<int>(floor(x));
}

// Clamp x to [min,max]
template <typename T, typename U, typename V>
inline T Clamp(const T& x, const U& min, const V& max) {
	return x < min ?
			static_cast<T>(min) :
			(x > max ? static_cast<T>(max) : x);
}

template <typename T>
inline T secsqr(const T& x) {
	double cosx = cos(x);
	return 1.0 / (cosx*cosx);
}

// Returns the sign of x
template <typename T>
int Sign(const T x) {
	return x < 0 ? -1 : (x > 0 ? 1 : 0);
}

// Returns 1 if x >= 0, otherwise returns -1
template <typename T>
int HalfSign(T x) {
	return x < 0 ? -1 : 1;
}

// Maps the range [-PI,PI] onto [0,PI]
template <typename T>
T UpAngle(const T theta) {
	return theta >= 0 ? theta : theta+M_PI;
}

// Maps the range [-PI,PI] onto (0,PI]
template <typename T>
T OpenUpAngle(T theta) {
	while (theta < 0) theta += M_PI;
	while (theta > M_PI-1e-5) theta -= M_PI;
	return theta;
}

inline byte ReverseBits(const byte b) {
	// God only knows how this works. Consult this page:
	// http://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith64Bits
	return ((b * 0x80200802ULL) & 0x0884422110ULL) * 0x0101010101ULL >> 32;
}

template <typename T>
T Ring(T theta, const T& max) {
	while (theta >= max) theta -= max;
	return theta;
}


// Returns the distance from a to b in a ring of length N:
// (0,1,...,N-1)
template <typename T>
inline T RingDist(const T& a, const T& b, const T& n) {
	const T d = abs(a-b);
	return min(d, n-d);
}
}
