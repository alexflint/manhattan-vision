/*
 * line_segment.h
 *
 *  Created on: 15 Jul 2010
 *      Author: alexf
 */

#pragma once

#include <iostream>

#include "geom_utils.h"
#include "common_types.h"

namespace indoor_context {

// Represents a 2D line segment in homogeneous coordinates
class LineSeg {
public:
	Vec3 start, end;

	inline LineSeg() : start(toon::Zeros), end(toon::Zeros) {
	}
	inline LineSeg(const Vec3& a, const Vec3& b) : start(a), end(b) {
	}
	// Get the homogeneous line equation
	inline Vec3 eqn() const {
		return start^end;
	}
	// Get the midpoint (in the Euclidean sense)
	inline Vec3 midpoint() const {
		return HMidpoint(start, end);
	}
};

}  // namespace indoor_context
