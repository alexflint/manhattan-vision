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
#include "manhattan_inference.h"
#include "map.h"
#include "map.pb.h"
#include "dp_mex_helpers.h"

using namespace indoor_context;
using namespace toon;
using boost::format;

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	InitMex();
  if (nlhs != 1 || nrhs < 1 || nrhs > 2) {
		mexErrMsgTxt("Usage: psi=compatibility(case, labels)");
	}

	// Get the case name
	string case_name = MatlabArrayToString(prhs[0]);
	DLOG << "Computing compatibility for case " << case_name;

	// Get the labels
	MatI labels;
	if (nrhs < 2) {
		DLOG << "Warning: initializing with dummy matrix filled with zeros";
		labels.Resize(480, 640, 0);
	} else {
		MatlabArrayToMatrix(prhs[1], labels);
	}

	// Compute psi
	ManhattanInference::PsiVec psi;
	ManhattanInference& inf = FrameStore::instance().Get(case_name);
	DLOG << "Computing psi...";
	inf.ComputePsi(labels, psi);
	DLOG << "Norm of psi: " << norm(psi);
	plhs[0] = NewMatlabArrayFromMatrix(psi.as_col());
}
