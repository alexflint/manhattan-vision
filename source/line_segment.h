/*
 * line_segment.h
 *
 *  Created on: 15 Jul 2010
 *      Author: alexf
 */

#pragma once

#include "common_types.h"

namespace indoor_context {
	// Represents a 2D line segment in homogeneous coordinates
	class LineSegment {
	public:
		Vec3 start, end;

		// Constructors
		inline LineSegment() : start(toon::Zeros), end(toon::Zeros) { }
		inline LineSegment(const Vec3& a, const Vec3& b) : start(a), end(b) { }
		// Get the homogeneous line equation
		inline Vec3 eqn() const { return start^end;	}
		// Get the (Euclidean) midpoint
		Vec3 midpoint() const;
		// Transform start and end point by a homography. This does no
		// clipping: for 3D transforms see clipping.h.
		LineSegment Transform(const Mat3& h) const;
	};

	// Backwards compatibility
	typedef LineSegment LineSeg;
}  // namespace indoor_context
