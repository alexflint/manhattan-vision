#include <string>

#include <boost/format.hpp>
#include <google/profiler.h>

#include "common_types.h"
#include "image_utils.h"
#include "map.h"
#include "map.pb.h"
#include "manhattan_dp.h"
#include "bld_helpers.h"
#include "dp_structures.h"
#include "matlab_utils.h"
#include "timer.h"

#include "counted_foreach.tpp"
#include "io_utils.tpp"
#include "vector_utils.tpp"

using namespace indoor_context;
using namespace toon;
using boost::format;

lazyvar<float> gvWallPenalty("ManhattanDP.DefaultWallPenalty");
lazyvar<float> gvOcclusionPenalty("ManhattanDP.DefaultOcclusionPenalty");

void StructToObjective(const mxArray* objstruct, DPObjective& obj) {
	ConstMatlabStructure objective = ObjectiveProto.From(objstruct);
	VecI dims = GetMatlabArrayDims(objective(0, "pixel_scores"));
	CHECK_EQ(dims.Size(), 3);
	CHECK_EQ(asToon(dims), makeVector(obj.ny(), obj.nx(), 3));

	double* pixeldata = mxGetPr(objective(0, "pixel_scores"));
	for (int i = 0; i < 3; i++) {
		// matlab arrays are stored column-major
		for (int x = 0; x < obj.nx(); x++) {
			for (int y = 0; y < obj.ny(); y++) {
				obj.pixel_scores[i][y][x] = *pixeldata++;
			}
		}
	}
}

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	if (nrhs != 3 || nlhs != 0) {
		mexErrMsgTxt("Usage: dp_check_features(frame, image_obj, grid_obj)\n");
	}

	// Construct the camera
	ConstMatlabStructure frame = FrameProto.From(prhs[0]);
	LinearCamera camera;
	PosedCamera pc;
	FrameStructToCamera(frame, camera, pc);

	// Construct geometry
	MatD mFtoC = MatlabArrayToMatrix(frame(0, "floor_to_ceil"));
	CHECK_EQ(matrix_size(mFtoC), makeVector(3,3));
	Mat3 fToC = asToon(mFtoC);
	DPGeometry geom(pc, fToC);

	// Extract the pixel scores (image coords)
	DPObjective image_obj(asToon(pc.image_size()));
	StructToObjective(prhs[1], image_obj);
	MonocularPayoffGen image_gen(image_obj, geom);
	DPObjective xf_image_obj(geom.grid_size);
	for (int i = 0; i < 3; i++) {
		geom.TransformDataToGrid(image_obj.pixel_scores[i], xf_image_obj.pixel_scores[i]);
	}

	// Extract the pixel scores (grid coords)
	DPObjective grid_obj(geom.grid_size);
	StructToObjective(prhs[2], grid_obj);
	MonocularPayoffGen grid_gen(grid_obj, geom);

	// Check for equality between objectives
	for (int i = 0; i < 3; i++) {
		CHECK_EQ(matrix_size(xf_image_obj.pixel_scores[i]), geom.grid_size);
		CHECK_EQ(matrix_size(grid_obj.pixel_scores[i]), geom.grid_size);
		for (int y = 0; y < geom.grid_size[1]; y++) {
			const float* imagerow = xf_image_obj.pixel_scores[i][y];
			const float* gridrow = grid_obj.pixel_scores[i][y];
			for (int x = 0; x < geom.grid_size[0]; x++) {
				double err;
				if (abs(imagerow[x]) < 1e-8) {
					err = abs(imagerow[x] - gridrow[x]);
				} else {
					err = abs(gridrow[x] / imagerow[x] - 1.0);
				}
				CHECK_LT(err, 1e-8)
					<< "Objectives differ at ("<<x<<","<<y<<"): pixeldata="<<imagerow[x]<<" griddata="<<gridrow[x];
			}
		}
	}

	// Check that they are equal
	for (int i = 0; i < 2; i++) {
		CHECK_EQ(matrix_size(grid_gen.payoffs.wall_scores[i]), geom.grid_size);
		CHECK_EQ(matrix_size(image_gen.payoffs.wall_scores[i]), geom.grid_size);
		for (int y = 0; y < geom.grid_size[1]; y++) {
			const float* gridrow = grid_gen.payoffs.wall_scores[i][y];
			const float* imagerow = image_gen.payoffs.wall_scores[i][y];
			for (int x = 0; x < geom.grid_size[0]; x++) {
				double err = abs(gridrow[x] / imagerow[x] - 1.0);
				CHECK_LT(err, 1e-8)
					<< "Payoffs differ at ("<<x<<","<<y<<"): pixeldata="<<imagerow[x]<<" griddata="<<gridrow[x];
			}
		}
	}
}
