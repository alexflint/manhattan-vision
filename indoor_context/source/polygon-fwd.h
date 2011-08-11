// Forward declarations for templates declared in polygon.tpp

#pragma once

#include "common_types.h"

namespace indoor_context {
	// Represents a polygon as a sequence of vertices
	template <unsigned N, typename T=double>
	class Polygon;

	// Represents an axis-aligned bounding box
	template <typename T=double>
	class Bounds2D;
}
