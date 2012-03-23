/*
 * matrix_traits.tpp
 * Provides compile-time inspection
 *
 *  Created on: 6 Aug 2010
 *      Author: alexf
 */

#pragma once

#include "common_types.h"

namespace indoor_context {
	// matrix_width
	template<typename T>
	int matrix_width(const toon::Matrix<toon::Dynamic, toon::Dynamic, T>& A) {
		return A.num_cols();
	}
	template<int M, int N, typename T, typename Layout>
	int matrix_width(const toon::Matrix<M,N,T,Layout>& A) {
		return N;
	}
	template<int M, int N, typename T>
	int matrix_width(const VNL::MatrixFixed<M,N,T>& A) {
		return N;
	}
	template<typename T>
	int matrix_width(const VNL::Matrix<T>& A) {
		return A.Cols();
	}
	template<typename T>
	int matrix_width(const T& A) {
		return A.nx();
	}


	// matrix_height
	template<typename T>
	int matrix_height(const toon::Matrix<toon::Dynamic, toon::Dynamic, T>& A) {
		return A.num_rows();
	}
	template<int M, int N, typename T, typename Layout>
	int matrix_height(const toon::Matrix<M,N,T,Layout>& A) {
		return M;
	}
	template<int M, int N, typename T>
	int matrix_height(const VNL::MatrixFixed<M,N,T>& x) {
		return M;
	}
	template<typename T>
	int matrix_height(const VNL::Matrix<T>& A) {
		return A.Rows();
	}
	template<typename T>
	int matrix_height(const T& A) {
		return A.ny();
	}
	



	// matrix_size
	template <typename T>
	Vec2I matrix_size(const T& A) {
		return toon::makeVector(matrix_width(A), matrix_height(A));
	}


	template <typename T>
	struct matrix_traits { };

	template <int Rows, int Cols, class Precision, class Layout>
	struct matrix_traits<toon::Matrix<Rows,Cols,Precision,Layout> > {
		typedef typename toon::Matrix<Rows,Cols,Precision,Layout> matrix_type;
		typedef Precision value_type;
		typedef Layout layout;
		static const int fixed_height = Rows;
		static const int fixed_width = Cols;
		static const bool is_fixed_size = true;
	};

	template <class Precision, class Layout>
	struct matrix_traits<toon::Matrix<-1,-1,Precision,Layout> > {
		typedef typename toon::Matrix<-1,-1,Precision,Layout> matrix_type;
		typedef Precision value_type;
		typedef Layout layout;
		static const int fixed_height = -1;
		static const int fixed_width = -1;
		static const bool is_fixed_size = false;
	};

	template <class Precision>
	struct matrix_traits<VNL::Matrix<Precision> > {
		typedef typename VNL::Matrix<Precision> matrix_type;
		typedef Precision value_type;
		static const int fixed_height = -1;
		static const int fixed_width = -1;
		static const bool is_fixed_size = false;
	};








	// Vector length
	template<typename T>
	int vector_length(const toon::Vector<toon::Dynamic, T>& A) {
		return A.size();
	}
	template<int N, typename T, typename Layout>
	int vector_length(const toon::Vector<N,T,Layout>& A) {
		return N;
	}
	template<int N, typename T>
	int vector_length(const VNL::VectorFixed<N,T>& x) {
		return N;
	}
	template<typename T>
	int vector_length(const VNL::Vector<T>& A) {
		return A.Size();
	}
	template<typename T>
	int vector_length(const std::vector<T>& v) {
		return v.size();
	}

	
	template <typename T>
	struct vector_traits { };

	template <int N, class Precision, class Layout>
	struct vector_traits<toon::Vector<N,Precision,Layout> > {
		typedef typename toon::Vector<N,Precision,Layout> vector_type;
		typedef Precision value_type;
		typedef Layout layout;
		static const int fixed_length = N;
		static const bool is_fixed_size = true;
	};

	template <class Precision, class Layout>
	struct vector_traits<toon::Vector<toon::Dynamic,Precision,Layout> > {
		typedef typename toon::Vector<toon::Dynamic,Precision,Layout> vector_type;
		typedef Precision value_type;
		typedef Layout layout;
		static const int fixed_length = -1;
		static const bool is_fixed_size = false;
	};

	template <class Precision>
	struct vector_traits<VNL::Vector<Precision> > {
		typedef typename VNL::Vector<Precision> vector_type;
		typedef Precision value_type;
		static const int fixed_length = -1;
		static const bool is_fixed_size = false;
	};

	template <class T>
	struct vector_traits<std::vector<T> > {
		typedef typename std::vector<T> vector_type;
		typedef T value_type;
		static const int fixed_length = -1;
		static const bool is_fixed_size = false;
	};
}  // namespace indoor_context
