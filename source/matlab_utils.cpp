/*
 * matlab_utils.h
 *
 *  Created on: 4 Aug 2010
 *      Author: alexf
 */

#include "matlab_utils.h"

#include <string>
#include <mex.h>
#include <boost/filesystem.hpp>

#include "common_types.h"
#include "vars.h"

namespace indoor_context {

void InitMex() {
	// Use exceptions so that MATLAB doesn't die
	AssertionManager::SetExceptionMode();

	// Fix up paths
	fs::path homedir;
	try {
		homedir = getenv("HOME");
	} catch (const std::exception& ex) {
		mexErrMsgTxt("Could not read PATH environment variable.\n");
	}

	const fs::path basedir = homedir / "Code/indoor_context";  // ugly hack
	const fs::path workingdir = basedir / "build";
	try {
		fs::current_path(workingdir);
	} catch (...) {
		mexPrintf("Could not chdir to %s\n", workingdir.string().c_str());
		mexErrMsgTxt("Exiting.\n");
	}

	// Load vars
	InitVars((basedir/"config/common.cfg").string());
}

string MatlabArrayToString(const mxArray* p) {
	char* z = mxArrayToString(p);
	assert(z != NULL);
	string s(z);
	mxFree(z);
	return s;
}

mxArray* NewMatlabArrayFromString(const string& s) {
	return mxCreateString(s.c_str());
}

void MatlabArrayToMatrix(const mxArray* p, MatD& m) {
	CHECK(mxIsNumeric(p));
	m.Resize(mxGetM(p), mxGetN(p));
	const double* pd = mxGetPr(p);
	CHECK(pd);
	for (int y = 0; y < m.Rows(); y++) {
		double* row = m[y];
		for (int x = 0; x < m.Cols(); x++) {
			row[x] = *pd++;
		}
	}
}

void MatlabArrayToMatrix(const mxArray* p, MatI& m) {
	CHECK(mxIsNumeric(p));
	m.Resize(mxGetM(p), mxGetN(p));
	const double* pd = mxGetPr(p);
	CHECK(pd);
	for (int y = 0; y < m.Rows(); y++) {
		int* row = m[y];
		for (int x = 0; x < m.Cols(); x++) {
			row[x] = static_cast<int>(*pd++);
		}
	}
}

VecD MatlabArrayToVector(const mxArray* p) {
	CHECK(mxIsNumeric(p));
	VecD v(mxGetM(p) * mxGetN(p));
	const double* pd = mxGetPr(p);
	CHECK(pd);
	for (int i = 0; i < v.Size(); i++) {
		v[i] = *pd++;
	}
	return v;
}

mxArray* NewMatlabArray(int d0) {
	mwSize dims[1] = {d0};
	return mxCreateNumericArray(1, dims, mxDOUBLE_CLASS, mxREAL);
}

mxArray* NewMatlabArray(int d0, int d1) {
	mwSize dims[2] = {d0, d1};
	return mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
}

mxArray* NewMatlabArray(int d0, int d1, int d2) {
	mwSize dims[3] = {d0, d1, d2};
	return mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
}

}  // namespace indoor_context
