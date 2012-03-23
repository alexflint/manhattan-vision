#pragma once

#include "vector_utils.h"

#include <TooN/se3.h>
#include <TooN/LU.h>

#include "common_types.h"

#include "numeric_utils.tpp"
#include "range_utils.tpp"
#include "vw_image.tpp"

namespace indoor_context {
	inline void toImageRef(const Vec2& v, ImageRef& p) {
		p.x = v[0];
		p.y = v[1];
	}

	inline void toImageRef(const Vec3& v, ImageRef& p) {
		p.x = v[0]/v[2];
		p.y = v[1]/v[2];
	}

	// Distance computations for CHECK_EQ
	template <int N, typename T>
	double err(const toon::Vector<N,T>& a, const toon::Vector<N,T>& b) {
		return norm_1(a-b);
	}

	// Distance computations for CHECK_EQ
	template <int M, int N, typename T, typename U>
	double err(const toon::Matrix<M,N,T>& a, const toon::Matrix<M,N,U>& b) {
		return norm_1(a-b);
	}

	// TODO: sort out this rounding stuff, do it one way consistently!
	template <typename T>
	ImageRef round_pos(const toon::Vector<2,T>& v) {
		return ImageRef(roundi(v[0]), roundi(v[1]));
	}

	template <typename T>
	ImageRef round_pos(const toon::Vector<3,T>& v) {
		return round_pos(project(v));
	}

	template <int N, typename T>
	toon::Vector<N,int> RoundVector(const toon::Vector<N,T>& v) {
		toon::Vector<N,int> u;
		for (int i = 0; i < N; i++) {
			u[i] = roundi(v[i]);
		}
		return u;
	}

	template <typename T>
	toon::Vector<-1,int> RoundVector(const toon::Vector<-1,T>& v) {
		toon::Vector<-1,int> u;
		for (int i = 0; i < v.size(); i++) {
			u[i] = roundi(v[i]);
		}
		return u;
	}

	// For speed:
	template <typename T>
	inline toon::Vector<1,int> RoundVector(const toon::Vector<1,T>& v) {
		return toon::makeVector(roundi(v[0]));
	}

	// For speed:
	template <typename T>
	inline toon::Vector<2,int> RoundVector(const toon::Vector<2,T>& v) {
		return toon::makeVector(roundi(v[0]), roundi(v[1]));
	}

	// For speed:
	template <typename T>
	inline toon::Vector<3,int> RoundVector(const toon::Vector<3,T>& v) {
		return toon::makeVector(roundi(v[0]), roundi(v[1]), roundi(v[2]));
	}

	// toon::Vector -> ImageRef
	template <typename T>
	inline ImageRef asIR(const toon::Vector<2,T>& v) {
		return ImageRef(roundi(v[0]), roundi(v[1]));
	}

	// ImageRef -> toon::Vector
	inline Vec2 asToon(const ImageRef& v) {
		return toon::makeVector(v.x, v.y);
	}
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
	template <typename T>
	toon::Vector<toon::Dynamic,T> concat(const toon::Vector<toon::Dynamic,T>& u,
																			 const toon::Vector<toon::Dynamic,T>& v) {
		toon::Vector<toon::Dynamic,T> r(u.size()+v.size());
		r.slice(0, u.size()) = u;
		r.slice(u.size(), v.size) = v;
		return r;
	}

	// Concatenate two vectors
	template <typename T, int N>
	toon::Vector<toon::Dynamic,T> concat(const toon::Vector<N,T>& u,
																			 const toon::Vector<toon::Dynamic,T>& v) {
		toon::Vector<toon::Dynamic,T> r(N+v.size());
		r.slice(0, N) = u;
		r.slice(N, v.size()) = v;
		return r;
	}

	// Concatenate two vectors
	template <typename T, int N>
	toon::Vector<toon::Dynamic,T> concat(const toon::Vector<toon::Dynamic,T>& u,
																			 const toon::Vector<N,T>& v) {
		toon::Vector<toon::Dynamic,T> r(u.size()+N);
		r.slice(0, u.size()) = u;
		r.slice(u.size(), N) = v;
		return r;
	}

	// Concatenate two vectors
	template <typename T, int M, int N>
	toon::Vector<M+N,T> concat(const toon::Vector<M,T>& u, const toon::Vector<N,T>& v) {
		toon::Vector<M+N,T> r;
		r.template slice<0,M>() = u;
		r.template slice<M,N>() = v;
		return r;
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
	template <typename T>
	inline toon::Vector<3,T> atretina(const toon::Vector<3,T>& x) {
		return toon::makeVector(x[0]/x[2], x[1]/x[2], 1.0);
	}

	template <typename T>
	inline toon::Vector<4,T> atretina(const toon::Vector<4,T>& x) {
		return toon::makeVector(x[0]/x[3], x[1]/x[3], x[2]/x[3], 1.0);
	}

	// Get the column of an SO3<>
	template <typename T>
	inline toon::Vector<3,T> col(const toon::SO3<T>& r, int i) {
		return r.get_matrix().T()[i];
	}

	// Get a unit vector with non-negative Z coordinate in the specified
	// direction
	template <typename T>
	inline toon::Vector<3,T> pve_unit(const toon::Vector<3,T>& x) {
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
		toon::Vector<N> v;
		for (int i = 0; i < N; i++) {
			v[i] = d*rand();
		}
		return v;
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

	template <typename T, int N>
	toon::Matrix<N,N,T> Inverse_LU(const toon::Matrix<N,N,T>& m) {
		return toon::LU<N,T>(m).get_inverse();
	}


	// Shift a matrix horizontally, copying pixels at the trailing
	template <typename T>
	void ShiftHoriz(const VNL::Matrix<T>& in,
									VNL::Matrix<T>& out,
									int dx) {
		out.Resize(in.Rows(), in.Cols());
		for (int y = 0; y < in.Rows(); y++) {
			const float* inrow = in[y];
			float* outrow = out[y];
			for (int x = 0; x < in.Cols(); x++) {
				outrow[x] = inrow[Clamp<int>(x+dx,0,in.Cols()-1)];
			}
		}
	}

	// Shift a matrix vertically, copying pixels at the trailing
	template <typename T>
	void ShiftVert(const VNL::Matrix<T>& in,
								 VNL::Matrix<T>& out,
								 int dy) {
		out.Resize(in.Rows(), in.Cols());
		for (int y = 0; y < in.Rows(); y++) {
			const float* inrow = in[Clamp<int>(y+dy,0,in.Rows()-1)];
			float* outrow = out[y];
			for (int x = 0; x < in.Cols(); x++) {
				outrow[x] = inrow[x];
			}
		}
	}
}  // namespace indoor_context
