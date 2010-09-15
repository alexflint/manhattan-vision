/*
 * matlab_utils.h
 *
 *  Created on: 4 Aug 2010
 *      Author: alexf
 */

#pragma once

#include <string>
#include <mex.h>
#include "common_types.h"
#include "matrix_traits.tpp"

namespace indoor_context {

// Helper to initialize vars etc for MEX files
void InitMex();

// Copy a matlab array to a toon array
	template <typename T>
T MatlabArrayToScalar(const mxArray* p) {
	CHECK(mxIsNumeric(p));
	CHECK_EQ(mxGetM(p), 1);
	CHECK_EQ(mxGetN(p), 1);
	return static_cast<T>(*mxGetPr(p));
}

// Copy a matlab string to a std::string
string MatlabArrayToString(const mxArray* p);

// Copy a string to a matlab array
mxArray* NewMatlabArrayFromString(const string& s);

// Copy a matlab array to a toon array
void MatlabArrayToMatrix(const mxArray* p, MatD& m);

// Copy a matlab array to a VNL matrix
void MatlabArrayToMatrix(const mxArray* p, MatI& m);

// Copy a matlab array to a VNL vector. Copies all elements row-by-row.
VecD MatlabArrayToVector(const mxArray* p);

// Convenience functions to allocate matlab arrays of 1, 2, or 3 dimensions
mxArray* NewMatlabArray(int d0);
mxArray* NewMatlabArray(int d0, int d1);
mxArray* NewMatlabArray(int d0, int d1, int d2);

// Copy a toon matrix to a matlab array
template <typename Matrix>
void MatrixToMatlabArray(const Matrix& m, mxArray* p) {
	typedef typename matrix_traits<Matrix>::value_type T;
	if (mxGetM(p) != m.num_rows()) {
		mxSetM(p, m.num_rows());
	}
	if (mxGetN(p) != m.num_cols()) {
		mxSetN(p, m.num_cols());
	}
	double* pd = mxGetPr(p);
	// Note that matlab matrices are column-major
	for (int x = 0; x < m.num_cols(); x++) {
		for (int y = 0; y < m.num_rows(); y++) {
			*pd++ = static_cast<double>(m[y][x]);
		}
	}
}

// Copy a toon matrix to a matlab array
template <typename T>
void MatrixToMatlabArray(const VNL::Matrix<T>& m, mxArray* p) {
	if (mxGetM(p) != m.Rows()) {
		mxSetM(p, m.Rows());
	}
	if (mxGetN(p) != m.Cols()) {
		mxSetN(p, m.Cols());
	}
	double* pd = mxGetPr(p);
	// Note that matlab matrices are column-major
	for (int x = 0; x < m.Cols(); x++) {
		for (int y = 0; y < m.Rows(); y++) {
			*pd++ = static_cast<double>(m[y][x]);
		}
	}
}

// Copy a toon matrix to a matlab array
template <typename Matrix>
mxArray* NewMatlabArrayFromMatrix(const Matrix& m) {
	mxArray* p = NewMatlabArray(m.num_rows(), m.num_cols());
	MatrixToMatlabArray(m, p);
	return p;
}

template <typename T>
mxArray* NewMatlabArrayFromMatrix(const VNL::Matrix<T>& m) {
	mxArray* p = NewMatlabArray(m.Rows(), m.Cols());
	MatrixToMatlabArray(m, p);
	return p;
}

}  // namespace indoor_context
