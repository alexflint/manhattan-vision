#pragma once

#include "matlab_structure.h"
#include "camera.h"

namespace indoor_context {
	// Represents a calibrated frame (similar to PosedImage)
	extern MatlabProto DetectionProto;
	// Represents an objective function optimized by ManhattanDP (similar to DPObjective)
	extern MatlabProto LineProto;
}
