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

#include "common_types.h"
#include "image_utils.h"
#include "matlab_utils.h"
#include "bld_helpers.h"
#include "manhattan_inference.h"
#include "map.h"
#include "map.pb.h"
#include "dp_mex_helpers.h"

#include "vector_utils.tpp"

using namespace toon;
using namespace indoor_context;
using boost::format;

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	InitMex();
	if (nlhs != 1 || nrhs != 2) {
		mexErrMsgTxt("Usage: orients=solve_dp(case, weights)\n");
	}

	// Deal with params
	string case_name = MatlabArrayToString(prhs[0]);
	VecD weights = MatlabArrayToVector(prhs[1]);
	CHECK_EQ(weights.Size(), ManhattanInference::kFeatureLength*3);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	KeyFrame& kf = LoadFrameByCaseName(case_name, map, gt_map);
	kf.LoadImage();

	// Compute features
	ManhattanInference inf;
	Mat3 floorToCeil = GetFloorCeilHomology(*kf.pc, gt_map.floorplan());
	mexPrintf("Performing inference...\n");
	inf.Compute(kf.image, floorToCeil, asToon(weights));

	// Copy the compat vector to a matlab array
	cout << "Copying to matlab format...\n";
	plhs[0] = NewMatlabArrayFromMatrix(inf.reconstructor.dp.soln_orients);
}
