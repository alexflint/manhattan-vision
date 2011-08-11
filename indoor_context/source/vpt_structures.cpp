#include "dp_structures.h"

#include <se3.h>

#include "common_types.h"
#include "matlab_utils.h"
#include "camera.h"

#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	MatlabProto DetectionProto("vpts",
														 "lines");

	MatlabProto LineProto("start",
												"end");
}
