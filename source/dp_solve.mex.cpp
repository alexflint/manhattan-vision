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
	if (nrhs < 2 || nrhs > 3) {
		mexErrMsgTxt("Usage: orients=dp_solve(case, objective[, include_image_orients?])\n");
	}
	ConstMatlabStructure frame = FrameProto.From(prhs[0]);

	// Construct the camera
	LinearCamera camera;
	PosedCamera pc;
	FrameStructToCamera(frame, camera, pc);

	// Construct geometry
	MatD mFtoC = MatlabArrayToMatrix(frame(0, "floor_to_ceil"));
	CHECK_EQ(matrix_size(mFtoC), makeVector(3,3));
	Mat3 fToC = asToon(mFtoC);
	DPGeometry geom(pc, fToC);

	// Extract the pixel-wise scores
	ConstMatlabStructure objective = ObjectiveProto.From(prhs[1]);
	VecI pix_dims = GetMatlabArrayDims(objective(0, "pixel_scores"));
	CHECK_EQ(pix_dims.Size(), 3);
	CHECK_EQ(asToon(pix_dims), makeVector(geom.ny(), geom.nx(), 3));

	DPObjective obj(geom.grid_size);
	double* pixeldata = mxGetPr(objective(0, "pixel_scores"));
	for (int i = 0; i < 3; i++) {
		// matlab arrays are stored column-major
		for (int x = 0; x < geom.nx(); x++) {
			for (int y = 0; y < geom.ny(); y++) {
				obj.pixel_scores[i][y][x] = *pixeldata++;
			}
		}
	}

	// Extract penalty terms -- must go above MonocularPayoffGen
	obj.wall_penalty = MatlabArrayToScalar(objective(0, "wall_penalty"));
	obj.occl_penalty = MatlabArrayToScalar(objective(0, "occlusion_penalty"));

	// Compile the pixel-wise scores to payoffs
	MonocularPayoffGen gen(obj, geom);

	// Extract the column-wise scores
	VecI wall_dims = GetMatlabArrayDims(objective(0, "wall_scores"));
	CHECK_EQ(wall_dims.Size(), 3);
	CHECK_EQ(asToon(wall_dims), makeVector(geom.ny(), geom.nx(), 2));
	double* walldata = mxGetPr(objective(0, "wall_scores"));
	for (int i = 0; i < 2; i++) {
		// matlab array are stored column-major
		for (int x = 0; x < geom.nx(); x++) {
			for (int y = 0; y < geom.ny(); y++) {
				gen.payoffs.wall_scores[i][y][x] += *walldata++;
			}
		}
	}

	// Do the reconstruction
	ManhattanDP dp;
	dp.Compute(gen.payoffs, geom);

	// Get the exact orientations in grid coordinates
	MatI grid_orients;
	dp.ComputeGridOrients(grid_orients);

	// Compute the path through the payoff matrix
	MatI soln_path;
	dp.ComputeSolutionPath(soln_path);

	// Create the solution
	MatlabStructure soln = SolutionProto.New(1);
	soln.put(0, "orients", NewMatlabArrayFromMatrix(grid_orients));
	soln.put(0, "path", NewMatlabArrayFromMatrix(soln_path));
	soln.put(0, "num_walls", NewMatlabArrayFromScalar(dp.soln_num_walls));
	soln.put(0, "num_occlusions", NewMatlabArrayFromScalar(dp.soln_num_occlusions));
	soln.put(0, "score", NewMatlabArrayFromScalar(dp.solution.score));

	// Copy the results back
	if (nlhs > 0) {
		plhs[0] = soln.get();
	}
}
