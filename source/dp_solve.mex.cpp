/*
 * compatibility.mex.cpp
 * Compute a compatibility vector between an image and a proposed reconstruction.
 * Serves as the "feature function" to SVM struct.
 *
 *  Created on: 4 Aug 2010
 *      Author: alexf
 */
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

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	TIMED("Complete MEX execution (inner)") {
		//ProfilerStart("/tmp/dp_solve.prof");
		InitMex();
		if (nlhs > 1 || nrhs != 2) {
			mexErrMsgTxt("Usage: orients=dp_solve(case, objective)\n");
		}

		ConstMatlabStructure frame = FrameProto.From(prhs[0]);
		ConstMatlabStructure objective = ObjectiveProto.From(prhs[1]);

		// Get camera matrices
		Mat3 intr = asToon(MatlabArrayToMatrix(frame(0, "camera_intrinsics")));
		Vec6 ln_extr = asToon(MatlabArrayToVector(frame(0, "camera_extrinsics")));
		Mat3 fToC = asToon(MatlabArrayToMatrix(frame(0, "floor_to_ceil")));

		// Build the posed image
		VecI dims = GetMatlabArrayDims(frame(0, "image"));
		LinearCamera camera(intr, ImageRef(dims[1], dims[0]));
		PosedCamera pc(SE3<>::exp(ln_extr), &camera);

		// Construct the objective function
		VecI obj_dims = GetMatlabArrayDims(objective(0, "scores"));
		CHECK_EQ(obj_dims, concat(3, dims));  // check that the scores are compatible with the image size
		DPObjective obj(dims[1], dims[0]);
		double* scoredata = mxGetPr(objective(0, "scores"));
		for (int i = 0; i < 3; i++) {
			// matlab arrays are stored column-major
			for (int x = 0; x < dims[1]; x++) {
				for (int y = 0; y < dims[0]; y++) {
					obj.pixel_scores[i][y][x] = *scoredata++;
				}
			}
		}
		obj.wall_penalty = MatlabArrayToScalar(objective(0, "wall_penalty"));
		obj.occl_penalty = MatlabArrayToScalar(objective(0, "occlusion_penalty"));
		CHECK_GE(obj.wall_penalty, 0);
		CHECK_GE(obj.occl_penalty, 0);
		DREPORT(obj.wall_penalty, obj.occl_penalty);

		// Do the reconstruction
		ManhattanDP dp;
		DPGeometry(pc, fToC);
		dp.Compute(obj, geom);

		// Create the solution
		MatlabStructure soln = SolutionProto.New(1);
		soln.put(0, "orients", NewMatlabArrayFromMatrix(dp.soln_orients));
		soln.put(0, "num_walls", NewMatlabArrayFromScalar(dp.soln_num_walls));
		soln.put(0, "num_occlusions", NewMatlabArrayFromScalar(dp.soln_num_occlusions));

		// Copy the results back
		if (nlhs > 0) {
			plhs[0] = soln.get();
		}
		//ProfilerStop();
	}
}
