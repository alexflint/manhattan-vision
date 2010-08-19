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
#include "manhattan_dp_features.h"
#include "map.h"
#include "map.pb.h"
#include "dp_mex_helpers.h"

using namespace toon;
using namespace indoor_context;
using boost::format;

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	InitMex();
	if (nlhs != 1 || nrhs != 2) {
		mexErrMsgTxt("Usage: psi=compatibility(case, labels)");
	}

	// Get the case name
	string case_name = MatlabArrayToString(prhs[0]);

	// Get the labels
	MatI labels;
	MatlabArrayToMatrix(prhs[1], labels);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	KeyFrame& kf = LoadFrameByCaseName(case_name, map, gt_map);
	kf.LoadImage();

	// Compute features
	cout << "Computing features...\n";
	CompatibilityFeatures ftrs;
	ftrs.Compute(kf.image, labels);

	// Copy the compat vector to a matlab array
	cout << "Copying to matlab format...\n";
	Matrix<CompatibilityFeatures::kFeatureLength*3,1> m = ftrs.compat.as_col();
	plhs[0] = NewMatlabArrayFromMatrix(ftrs.compat.as_col());
}
