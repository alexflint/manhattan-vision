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

using namespace indoor_context;
using namespace toon;
using boost::format;

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	InitMex();
	if (nlhs > 2 || nrhs < 1 || nrhs > 3) {
		mexErrMsgTxt("Usage: [orients gt_orients] = "
								 "dp_solve(case, weights, do_most_violated)\n");
	}

	// Deal with params
	string case_name = MatlabArrayToString(prhs[0]);
	DLOG << "Performing inference for for case " << case_name;

	VecD weights;
	if (nrhs < 2) {
		DLOG << "Warning: initializing with dummy weight vector filled with zeros";
		weights.Resize(ManhattanInference::kFeatureLength*3, 0);
	} else {
		weights = MatlabArrayToVector(prhs[1]);
	}
	toon::Vector<> w = asToon(weights);
	CHECK_EQ(weights.Size(), ManhattanInference::kFeatureLength*3);

	bool find_most_violated = false;
	if (nrhs >= 3) {
		find_most_violated = MatlabArrayToScalar<bool>(prhs[2]);
	}

	// Get the frame data
	ManhattanInference& inf = FrameStore::instance().Get(case_name);
	CHECK(inf.input_image) << "Case " << case_name << " not initialized. "
		"Please make sure to call dp_load_cases first";

	// Do inference
	if (find_most_violated) {
		DLOG << "Performing most violated constraint inference...\n";
		inf.ComputeMostViolated(w);
	} else {
		DLOG << "Performing test-time inference...\n";
		inf.ComputeReconstruction(w);
	}

	// Copy the result back to matlab format
	plhs[0] = NewMatlabArrayFromMatrix(inf.reconstructor.dp.soln_orients);
	if (nrhs > 1) {
		plhs[1] = NewMatlabArrayFromMatrix(inf.gt_labels);
	}
}
