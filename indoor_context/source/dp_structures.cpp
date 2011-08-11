#include "dp_structures.h"

#include <se3.h>

#include "common_types.h"
#include "matlab_utils.h"
#include "camera.h"

#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	MatlabProto FrameProto("image",
												 "camera_intrinsics",
												 "camera_extrinsics",
												 "floor_to_ceil");

	MatlabProto ObjectiveProto("pixel_scores",
														 "wall_scores",
														 "wall_penalty",  // a negative number, multiplied by num_walls
														 "occlusion_penalty"
														 );

	MatlabProto CaseProto("frame_id",
												"sequence_name",
												"frame",
												"image_file",
												"pixel_features",
												//"full_pixel_features",
												"wall_features",
												"ground_truth");

	MatlabProto SolutionProto("orients",
														"num_walls",  // _total_ number of wall intersections
														"num_occlusions",  // number of the intersections above that are occluding
														"score",  // score computed through the payoff matrix
														"path" // solution represented as a path through the payoff matrix
														);

	MatlabProto MetaProto("feature_strings");

	void FrameStructToCamera(const ConstMatlabStructure& frame,
													 LinearCamera& camera,
													 PosedCamera& pc) {
		VecI dims = GetMatlabArrayDims(frame(0, "image"));
		camera.SetIntrinsics(asToon(MatlabArrayToMatrix(frame(0, "camera_intrinsics"))));
		camera.SetImageSize(ImageRef(dims[1], dims[0]));
		pc.SetPose(asToon(MatlabArrayToVector(frame(0, "camera_extrinsics"))));
		pc.SetCamera(&camera);
	}
}
