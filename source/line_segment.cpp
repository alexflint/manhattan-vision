#include "line_segment.h"
#include "geom_utils.h"

namespace indoor_context {
	Vec3 LineSeg::midpoint() const {
		return HMidpoint(start, end);
	}

	LineSeg LineSeg::Transform(const Mat3& h) const {
		return LineSeg(h*start, h*end);
	}
}
