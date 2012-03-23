#include "line_segment.h"
#include "geom_utils.h"

namespace indoor_context {
	Vec3 LineSegment::midpoint() const {
		return HMidpoint(start, end);
	}

	LineSeg LineSegment::Transform(const Mat3& h) const {
		return LineSegment(h*start, h*end);
	}
}
