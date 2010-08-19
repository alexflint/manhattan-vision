#pragma once

#include <boost/static_assert.hpp>
#include <boost/scoped_array.hpp>

#include "common_types.h"

namespace indoor_context {

// Represents a table with the number of dimensions set at compile
// time and the length of each dimension set at runtime.
template <unsigned N, typename T>
class Table {
public:
	typedef T* iterator;
	typedef T* const_iterator;
	// Constructors
	Table() {	}
	Table(int dim1) {
		BOOST_STATIC_ASSERT(N == 1);
		Init(toon::makeVector(dim1));
	}
	Table(int dim1, int dim2) {
		BOOST_STATIC_ASSERT(N == 2);
		Init(toon::makeVector(dim1, dim2));
	}
	Table(int dim1, int dim2, int dim3) {
		BOOST_STATIC_ASSERT(N == 3);
		Init(toon::makeVector(dim1, dim2, dim3));
	}
	Table(int dim1, int dim2, int dim3, int dim4) {
		BOOST_STATIC_ASSERT(N == 4);
		Init(toon::makeVector(dim1, dim2, dim3, dim4));
	}
	Table(int dim1, int dim2, int dim3, int dim4, int dim5) {
		BOOST_STATIC_ASSERT(N == 5);
		Init(toon::makeVector(dim1, dim2, dim3, dim4, dim5));
	}
	Table(int dim1, int dim2, int dim3, int dim4, int dim5, int dim6) {
		BOOST_STATIC_ASSERT(N == 6);
		Init(toon::makeVector(dim1, dim2, dim3, dim4, dim5, dim6));
	}

	// Set the size of the table
	void Resize(int dim1) {
		BOOST_STATIC_ASSERT(N == 1);
		Init(toon::makeVector(dim1));
	}
	void Resize(int dim1, int dim2) {
		BOOST_STATIC_ASSERT(N == 2);
		Init(toon::makeVector(dim1, dim2));
	}
	void Resize(int dim1, int dim2, int dim3) {
		BOOST_STATIC_ASSERT(N == 3);
		Init(toon::makeVector(dim1, dim2, dim3));
	}
	void Resize(int dim1, int dim2, int dim3, int dim4) {
		BOOST_STATIC_ASSERT(N == 4);
		Init(toon::makeVector(dim1, dim2, dim3, dim4));
	}
	void Resize(int dim1, int dim2, int dim3, int dim4, int dim5) {
		BOOST_STATIC_ASSERT(N == 5);
		Init(toon::makeVector(dim1, dim2, dim3, dim4, dim5));
	}
	void Resize(int dim1, int dim2, int dim3, int dim4, int dim5, int dim6) {
		BOOST_STATIC_ASSERT(N == 6);
		Init(toon::makeVector(dim1, dim2, dim3, dim4, dim5, dim6));
	}

	// Element accessors
	T& operator()(int x1) {
		BOOST_STATIC_ASSERT(N == 1);
		return data[ x1*w[0] ];
	}
	T& operator()(int x1, int x2) {
		BOOST_STATIC_ASSERT(N == 2);
		return data[ x1*w[0] + x2*w[1] ];
	}
	T& operator()(int x1, int x2, int x3) {
		BOOST_STATIC_ASSERT(N == 3);
		return data[ x1*w[0] + x2*w[1] + x3*w[2] ];
	}
	T& operator()(int x1, int x2, int x3, int x4) {
		BOOST_STATIC_ASSERT(N == 4);
		return data[ x1*w[0] + x2*w[1] + x3*w[2] + x4*w[3] ];
	}
	T& operator()(int x1, int x2, int x3, int x4, int x5) {
		BOOST_STATIC_ASSERT(N == 5);
		return data[ x1*w[0] + x2*w[1] + x3*w[2] + x4*w[3]+ x5*w[4] ];
	}
	T& operator()(int x1, int x2, int x3, int x4, int x5, int x6) {
		BOOST_STATIC_ASSERT(N == 6);
		return data[ x1*w[0] + x2*w[1] + x3*w[2] + x4*w[3]+ x5*w[4] + x6*w[5] ];
	}

	// Const element accessors
	const T& operator()(int x1) const {
		BOOST_STATIC_ASSERT(N == 1);
		return data[ x1*w[0] ];
	}
	const T& operator()(int x1, int x2) const {
		BOOST_STATIC_ASSERT(N == 2);
		return data[ x1*w[0] + x2*w[1] ];
	}
	const T& operator()(int x1, int x2, int x3) const {
		BOOST_STATIC_ASSERT(N == 3);
		return data[ x1*w[0] + x2*w[1] + x3*w[2] ];
	}
	const T& operator()(int x1, int x2, int x3, int x4) const {
		BOOST_STATIC_ASSERT(N == 4);
		return data[ x1*w[0] + x2*w[1] + x3*w[2] + x4*w[3] ];
	}
	const T& operator()(int x1, int x2, int x3, int x4, int x5) const {
		BOOST_STATIC_ASSERT(N == 5);
		return data[ x1*w[0] + x2*w[1] + x3*w[2] + x4*w[3] + x5*w[4] ];
	}
	const T& operator()(int x1, int x2, int x3, int x4, int x5, int x6) const {
		BOOST_STATIC_ASSERT(N == 6);
		return data[ x1*w[0] + x2*w[1] + x3*w[2] + x4*w[3] + x5*w[4] + x6*w[5] ];
	}

	// Fill the table with a specified value
	void Fill(const T& x) {
		fill(data.get(), data.get()+n, x);
	}

	// Get length of dimensions
	inline toon::Vector<N, int> dimensions() const { return dim_lens; }

	// Get the i-th dimension
	inline int dim(int i) const { return dim_lens[i]; }

	// Get size
	int size() const { return n; }

	// Iterator ranges
	iterator begin() {
		return data.get();
	}
	iterator end() {
		return data.get()+n;
	}
	const_iterator begin() const {
		return data.get();
	}
	const_iterator end() const {
		return data.get()+n;
	}
private:
	int n;
	toon::Vector<N, int> dim_lens;
	toon::Vector<N, int> w;
	scoped_array<T> data;

	// Initialize the table.
	void Init(toon::Vector<N, int> dims) {
		dim_lens = dims;
		w[N-1] = 1;
		for (int i = N-2; i >= 0; i--) {
			w[i] = w[i+1] * dim_lens[i+1];
		}
		n = w[0]*dim_lens[0];
		data.reset(new T[n]);
	}
};

}
