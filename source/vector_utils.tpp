/*
 * vector_utils.tpp
 *
 *  Created on: 2 Jun 2010
 *      Author: alexf
 */
#pragma once

#include "TooN/se3.h"

#include "common_types.h"
#include "map.pb.h"

#include "range_utils.tpp"

namespace indoor_context {

// defined in math_utils.cpp TODO: move these to .h
toon::Vector<2> asToon(const proto::Vec2& x);
toon::Vector<3> asToon(const proto::Vec3& x);
toon::Vector<4> asToon(const proto::Vec4& x);
toon::Vector<5> asToon(const proto::Vec5& x);
toon::Vector<6> asToon(const proto::Vec6& x);
proto::Vec2 asProto(const toon::Vector<2>& x);
proto::Vec3 asProto(const toon::Vector<3>& x);
proto::Vec4 asProto(const toon::Vector<4>& x);
proto::Vec5 asProto(const toon::Vector<5>& x);
proto::Vec6 asProto(const toon::Vector<6>& x);

// Construct a full 3x4 matrix representing a rigid 3D transform
toon::Matrix<3,4> as_matrix(const toon::SE3<>& se3);

template <typename T>
ImageRef round_pos(const toon::Vector<2,T>& v) {
	return ImageRef(roundi(v[0]), roundi(v[1]));
}

template <typename T>
ImageRef round_pos(const toon::Vector<3,T>& v) {
	return round_pos(project(v));
}

// toon::Vector -> ImageRef
template <typename T>
ImageRef asIR(const toon::Vector<2,T>& v) {
	return ImageRef(roundi(v[0]), roundi(v[1]));
}

// ImageRef -> toon::Vector
toon::Vector<2> asToon(const ImageRef& ir);



// VNL -> toon for fixed size vectors
template <int N, typename T>
toon::Vector<N,T> asToon(const VNL::VectorFixed<N,T>& v) {
	toon::Vector<N,T> u;
	for (int i = 0; i < N; i++) {
		u[i] = v[i];
	}
	return u;
}

// toon -> VNL for fixed size vectors
template <int N, typename T>
VNL::VectorFixed<N,T> asVNL(const toon::Vector<N,T>& v) {
	VNL::VectorFixed<N,T> u;
	for (int i = 0; i < N; i++) {
		u[i] = v[i];
	}
	return u;
}

// VNL -> toon for dynamic size vectors
template <typename T>
toon::Vector<toon::Dynamic,T> asToon(const VNL::Vector<T>& v) {
	toon::Vector<toon::Dynamic,T> u(v.Size());
	for (int i = 0; i < v.Size(); i++) {
		u[i] = v[i];
	}
	return u;
}

// toon -> VNL for dynamic size vectors
template <typename T>
VNL::Vector<T> asVNL(const toon::Vector<toon::Dynamic,T>& v) {
	VNL::Vector<T> u(v.size());
	for (int i = 0; i < v.size(); i++) {
		u[i] = v[i];
	}
	return u;
}

// VNL -> toon for fixed size matrices
template <int N, int M, typename T>
toon::Matrix<N,M,T> asToon(const VNL::MatrixFixed<N,M,T>& m) {
	toon::Matrix<N,M,T> u;
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) {
			u[i][j] = m[i][j];
		}
	}
	return u;
}

// toon -> VNL for fixed size matrices
template <int N, int M, typename T>
VNL::MatrixFixed<N,M,T> asVNL(const toon::Matrix<N,M,T>& v) {
	VNL::MatrixFixed<N,M,T> u;
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) {
			u[i][j] = v[i][j];
		}
	}
	return u;
}

// VNL -> toon for dynamic size matrices
template <typename T>
toon::Matrix<toon::Dynamic,toon::Dynamic,T> asToon(const VNL::Matrix<T>& v) {
	toon::Matrix<toon::Dynamic,toon::Dynamic,T> u(v.Rows(), v.Cols());
	for (int i = 0; i < v.Rows(); i++) {
		for (int j = 0; j < v.Cols(); j++) {
			u[i][j] = v[i][j];
		}
	}
	return u;
}

// toon -> VNL for dynamic size matrices
template <typename T>
VNL::Matrix<T> asVNL(const toon::Matrix<toon::Dynamic,toon::Dynamic,T>& v) {
	VNL::Matrix<T> u(v.num_rows(), v.num_cols());
	for (int i = 0; i < u.Rows(); i++) {
		for (int j = 0; j < u.Cols(); j++) {
			u[i][j] = v[i][j];
		}
	}
	return u;
}

// Concatenate two vectors
template <typename T, int M, int N>
toon::Vector<M+N,T> concat(const toon::Vector<M,T>& u, const toon::Vector<N,T>& v) {
	toon::Vector<M+N,T> r;
	r.template slice<0,M>() = u;
	r.template slice<M,N>() = v;
	return ;
}

// Append a scalar to the end of a vector
template <typename U, typename T, int N>
toon::Vector<N+1,T> concat(const toon::Vector<N,T>& v, const U& x) {
	toon::Vector<N+1,T> r;
	r.template slice<0,N>() = v;
	r[N] = x;
	return r;
}

// Append a scalar to the beginning of a vector
template <typename U, typename T, int N>
toon::Vector<N+1,T> concat(const U& x, const toon::Vector<N,T>& v) {
	toon::Vector<N+1,T> r;
	r.template slice<1,N>() = v;
	r[0] = x;
	return r;
}

// Divide the elements of a vector by the last element, as in
// unproject(project(v)).
inline toon::Vector<3> atretina(const toon::Vector<3>& x) {
	return toon::makeVector(x[0]/x[2], x[1]/x[2], 1.0);
}

inline toon::Vector<4> atretina(const toon::Vector<4>& x) {
	return toon::makeVector(x[0]/x[3], x[1]/x[3], x[2]/x[3], 1.0);
}

// Get the column of an SO3<>
template <typename T>
toon::Vector<3,T> col(const toon::SO3<T>& r, int i) {
	return r.get_matrix().T()[i];
}

// Get a unit vector with positive Z coordinate in the specified
// direction
template <typename T>
toon::Vector<3,T> pve_unit(const toon::Vector<3,T>& x) {
	return x * Sign(x[2]) / norm(x);
}

// Get the i-th axis in an N-coord system
template <unsigned N>
inline toon::Vector<N> GetAxis(const int i) {
	toon::Vector<N> v = toon::Zeros;
	v[i] = 1;
	return v;
}

// Generate a vector of random numbers
template <unsigned N>
toon::Vector<N> RandomVector() {
	double d = 1.0 / RAND_MAX;
	return toon::makeVector(d*rand(), d*rand(), d*rand());
}

// Computes a bounding box for a sequence of vectors. It is assumed that the range
// constaints toon::Vector
template <int N, typename Range>
pair<toon::Vector<N>, toon::Vector<N> >
ComputeBounds(const Range& vs) {
	typedef toon::Vector<N> VecN;
	VecN a = *begin(vs);
	VecN b = a;
	int d = begin(vs)->size();
	BOOST_FOREACH(const VecN& v, vs) {
		for (int i = 0; i < d; i++) {
			a[i] = min(a[i], v[i]);
			b[i] = max(b[i], v[i]);
		}
	}
	return make_pair(a, b);
}
}  // namespace indoor_context
