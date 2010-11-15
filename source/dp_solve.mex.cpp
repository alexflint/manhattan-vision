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
	InitMex();
	if (nrhs != 2) {
		mexErrMsgTxt("Usage: orients=dp_solve(case, objective)\n");
	}

	// Construct the camera
	ConstMatlabStructure frame = FrameProto.From(prhs[0]);
	PosedCamera pc = MakeCamera(frame);
	Mat3 fToC = asToon(MatlabArrayToMatrix(frame(0, "floor_to_ceil")));

	// Construct the objective function
	ConstMatlabStructure objective = ObjectiveProto.From(prhs[1]);
	VecI obj_dims = GetMatlabArrayDims(objective(0, "scores"));
	CHECK_EQ(obj_dims[0], pc.ny());  // check that the scores are compatible with the image size
	CHECK_EQ(obj_dims[1], pc.nx());
	CHECK_EQ(obj_dims[2], 3);
	DPObjective obj(asToon(pc.image_size()));
	double* scoredata = mxGetPr(objective(0, "scores"));
	for (int i = 0; i < 3; i++) {
		// matlab arrays are stored column-major
		for (int x = 0; x < pc.nx(); x++) {
			for (int y = 0; y < pc.ny(); y++) {
				obj.pixel_scores[i][y][x] = *scoredata++;
			}
		}
	}
	obj.wall_penalty = MatlabArrayToScalar(objective(0, "wall_penalty"));
	obj.occl_penalty = MatlabArrayToScalar(objective(0, "occlusion_penalty"));
	CHECK_GE(obj.wall_penalty, 0);
	CHECK_GE(obj.occl_penalty, 0);

	// Do the reconstruction
	DPGeometry geom(&pc, fToC);
	MonocularPayoffGen gen(obj, geom);
	ManhattanDP dp;
	dp.Compute(gen.payoffs, geom);

	// Compute the path through the payoff matrix
	MatI soln_path;
	dp.ComputeSolutionPath(soln_path);

	// Create the solution
	MatlabStructure soln = SolutionProto.New(1);
	soln.put(0, "orients", NewMatlabArrayFromMatrix(dp.soln_orients));
	soln.put(0, "num_walls", NewMatlabArrayFromScalar(dp.soln_num_walls));
	soln.put(0, "num_occlusions", NewMatlabArrayFromScalar(dp.soln_num_occlusions));
	soln.put(0, "payoffs0", NewMatlabArrayFromMatrix(gen.payoffs.wall_scores[0]));
	soln.put(0, "payoffs1", NewMatlabArrayFromMatrix(gen.payoffs.wall_scores[1]));
	soln.put(0, "path", NewMatlabArrayFromMatrix(soln_path));

	// Copy the results back
	if (nlhs > 0) {
		plhs[0] = soln.get();
	}
}
