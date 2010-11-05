#pragma once

#include "matlab_structure.h"

namespace indoor_context {
	// Represents a calibrated frame (similar to PosedImage)
	MatlabProto FrameProto("image",
												 "camera_intrinsics",
												 "camera_extrinsics",
												 "floor_to_ceil");

	// Represents an objective function optimized by ManhattanDP (similar to DPObjective)
	MatlabProto ObjectiveProto("scores",
														 "wall_penalty",  // a negative number, multiplied by num_walls
														 "occlusion_penalty"
														 );

	// Represents a training example
	MatlabProto CaseProto("frame_id",
												"sequence_name",
												"frame",
												"image_file",
												"features",
												"ground_truth");

	// Represents a solution to a reconstruction problem
	MatlabProto SolutionProto("orients",
														"num_walls",  // _total_ number of wall intersections
														"num_occlusions",  // number of the intersections above that are occluding
														"payoffs",  // payoff matrix computed during reconstruction
														"path" // solution represented as a path through the payoff matrix
														);
}
