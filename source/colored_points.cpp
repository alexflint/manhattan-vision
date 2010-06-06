/*
 * colored_points.cpp
 *
 *  Created on: 14 May 2010
 *      Author: alexf
 */

#include "colored_points.h"
#include "common_types.h"

namespace indoor_context {

ColoredPoints::ColoredPoints() : pointSize(2.0) {
}

void ColoredPoints::Add(const Vec3& v, const PixelRGB<byte>& color) {
	vs.push_back(make_pair(v, color));
}

}
