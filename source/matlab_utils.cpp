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


int GetFieldNumOrDie(const mxArray* array, const char* fieldName) {
	CHECK_NOT_NULL(array);
	int n = mxGetFieldNumber(array, fieldName);
	CHECK_GE(n, 0) << "No such field: '" << fieldName << "'";
	return n;
}


mxArray* GetFieldOrDie(const mxArray* array, mwIndex index, int field) {
	CHECK_NOT_NULL(array);
	mxArray* m = mxGetFieldByNumber(array, index, field);
	CHECK_NOT_NULL(m) << "Invalid field number: " << field;
	return m;
}

double MatlabArrayToScalar(const mxArray* p) {
	CHECK(mxIsNumeric(p));
	CHECK_EQ(mxGetM(p), 1);
	CHECK_EQ(mxGetN(p), 1);
	return *mxGetPr(p);
}

string MatlabArrayToString(const mxArray* p) {
	char* z = mxArrayToString(p);
	assert(z != NULL);
	string s(z);
	mxFree(z);
	return s;
}

VecI GetMatlabArrayDims(const mxArray* m) {
	mwSize n = mxGetNumberOfDimensions(m);
	const mwSize* size = mxGetDimensions(m);
	VecI dims(n);
	for (int i = 0; i < n; i++) {
		dims[i] = size[i];
	}
	return dims;
}

void MatlabArrayToImage(const mxArray* m, ImageRGB<byte>& image) {
	CHECK(mxIsDouble(m)) << "Only images of double type are supported at present";
	VecI dims = GetMatlabArrayDims(m);
	CHECK_EQ(dims.Size(), 3) << "MatlabArrayToImage expects a H x W x 3 array";
	CHECK_EQ(dims[2], 3) << "MatlabArrayToImage expects a H x W x 3 array";
	image.AllocImageData(dims[1], dims[0]);
	double* p = mxGetPr(m);
	for (int x = 0; x < dims[1]; x++)
		for (int y = 0; y < dims[0]; y++)
			image[y][x].r = *p++ * 255;  // pixels of type double are in [0,1] under matlab
	for (int x = 0; x < dims[1]; x++)
		for (int y = 0; y < dims[0]; y++)
			image[y][x].g = *p++ * 255;  // pixels of type double are in [0,1] under matlab
	for (int x = 0; x < dims[1]; x++)
		for (int y = 0; y < dims[0]; y++)
			image[y][x].b = *p++ * 255;  // pixels of type double are in [0,1] under matlab
	for (int x = 0; x < dims[1]; x++)
		for (int y = 0; y < dims[0]; y++)
			image[y][x].alpha = 0;
}

mxArray* NewMatlabArrayFromScalar(double x) {
	mxArray* p = NewMatlabArray(1,1);
	*mxGetPr(p) = x;
	return p;
}

mxArray* NewMatlabArrayFromImage(const ImageRGB<byte>& image) {
	mxArray* m = NewMatlabArray(image.GetHeight(), image.GetWidth(), 3);
	// matlab arrays are column-major
	// note that matlab interprets images of type double as values in [0,1] not [0,255]
	double* p = mxGetPr(m);
	for (int x = 0; x < image.GetWidth(); x++)
		for (int y = 0; y < image.GetHeight(); y++)
			*p++ = 1.0 * image[y][x].r / 255;
	for (int x = 0; x < image.GetWidth(); x++)
		for (int y = 0; y < image.GetHeight(); y++)
			*p++ = 1.0 * image[y][x].g / 255;
	for (int x = 0; x < image.GetWidth(); x++)
		for (int y = 0; y < image.GetHeight(); y++)
			*p++ = 1.0 * image[y][x].b / 255;
	return m;
}

mxArray* NewMatlabArrayFromString(const string& s) {
	return mxCreateString(s.c_str());
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
