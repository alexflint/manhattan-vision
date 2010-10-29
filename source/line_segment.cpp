#include "line_segment.h"
#include "geom_utils.h"

namespace indoor_context {
	Vec3 LineSeg::midpoint() const {
		return HMidpoint(start, end);
	}
}
