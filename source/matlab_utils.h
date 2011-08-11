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
	// Helper to initialize logging, exceptions, and gvars for MEX files
	// This is called automatically from mex_stub.cpp
	void InitMex();

	// Get a field number by name, or exit if it doesn't exist.
	int GetFieldNumOrDie(const mxArray* array, const char* fieldName);

	// Get a field value, or exit if it doesn't exist.
	mxArray* GetFieldOrDie(const mxArray* matrix, mwIndex index, int field);

	// Get the size of a matlab array
	VecI GetMatlabArrayDims(const mxArray* m);

	// Copy a matlab array to a toon array
	double MatlabArrayToScalar(const mxArray* p);

	// Copy a matlab string to a std::string
	string MatlabArrayToString(const mxArray* p);

	// Copy a H x W x 3 matlab array to an RGB image.
	void MatlabArrayToImage(const mxArray* m, ImageRGB<byte>& image);

	// Copy a string to a matlab array
	mxArray* NewMatlabArrayFromString(const string& s);

	// Allocate a 1x1 matlab array and initialize it to a scalar value
	mxArray* NewMatlabArrayFromScalar(double x);

	// Allocate a H x W x 3 matlab array and initialize it with an RGB image
	mxArray* NewMatlabArrayFromImage(const ImageRGB<byte>& image);

	// Copy a matlab array to a VNL vector. Copies all elements row-by-row.
	VecD MatlabArrayToVector(const mxArray* p);

	// Convenience functions to allocate matlab arrays of 1, 2, or 3 dimensions
	mxArray* NewMatlabArray(int d0);
	mxArray* NewMatlabArray(int d0, int d1);
	mxArray* NewMatlabArray(int d0, int d1, int d2);

	// Convert a matlab array to a matrix
	MatD MatlabArrayToMatrix(const mxArray* p);

	// Copy a toon matrix to a matlab array
	template <typename Matrix>
	void MatrixToMatlabArray(const Matrix& m, mxArray* p) {
		int ny = matrix_height(m);
		int nx = matrix_width(m);
		if (mxGetM(p) != ny) {
			mxSetM(p, ny);
		}
		if (mxGetN(p) != nx) {
			mxSetN(p, nx);
		}
		double* pd = mxGetPr(p);
		CHECK(pd);
		// matlab arrays are stored column-major
		for (int x = 0; x < nx; x++) {
			for (int y = 0; y < ny; y++) {
				*pd++ = static_cast<double>(m[y][x]);
			}
		}
	}

	// Copy a generic matrix to a matlab array
	template <typename Matrix>
	mxArray* NewMatlabArrayFromMatrix(const Matrix& m) {
		mxArray* p = NewMatlabArray(matrix_height(m), matrix_width(m));
		MatrixToMatlabArray(m, p);
		return p;
	}

	// Copy a matlab array to a VNL matrix
	template <typename T>
	void MatlabArrayToMatrix(const mxArray* p, VNL::Matrix<T>& m) {
		CHECK(mxIsNumeric(p));
		m.Resize(mxGetM(p), mxGetN(p));
		const double* pd = mxGetPr(p);
		CHECK(pd);
		// matlab arrays are stored column-major
		for (int x = 0; x < m.Cols(); x++) {
			for (int y = 0; y < m.Rows(); y++) {
				m[y][x] = static_cast<T>(*pd++);
			}
		}
	}
}  // namespace indoor_context
