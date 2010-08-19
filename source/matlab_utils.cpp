/*
 * matlab_utils.h
 *
 *  Created on: 4 Aug 2010
 *      Author: alexf
 */

#include "matlab_utils.h"

#include <string>
#include <mex.h>

#include "common_types.h"
#include "vars.h"

namespace indoor_context {

static const string kWorkingDir = "/homes/50/alexf/work/indoor_context/build";  // ugly hack

void InitMex() {
	AssertionManager::SetExceptionMode();

	// This is all horribly hacky and un-portable
	if (chdir(kWorkingDir.c_str())) {  // ugly hack!
		mexPrintf("Could not chdir to %s\n", kWorkingDir.c_str());
		mexErrMsgTxt("Could not chdir, exiting.");
	}
	scoped_array<char> cwd(get_current_dir_name());
	cout << "changed working dir to " << cwd.get() << endl;

	InitVars();
}

string MatlabArrayToString(const mxArray* p) {
	char* z = mxArrayToString(p);
	assert(z != NULL);
	string s(z);
	mxFree(z);
	return s;
}

mxArray* StringToMatlabArray(const string& s) {
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

/*mxArray* NewMatlabArrayFromTable(const Table<2, ManhattanDPFeatures::Feature >& table) {
	mwSize dims[3] = { table.dim(0), table.dim(1), table(0,0).size() };
	mxArray* arr = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
	double* pd = mxGetPr(arr);
	Table<2, ManhattanDPFeatures::Feature>::const_iterator it;
	for (it = table.begin(); it != table.end(); it++) {
		for (int i = 0; i < it->size(); i++) {
			*pd++ = (*it)[i];
		}
	}
	return arr;
}*/

}  // namespace indoor_context
