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
	template<typename T, typename U>
	int matrix_width(const VW::ImageBase<T,U>& A) {
		return A.GetWidth();
	}
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
	template<typename T, typename U>
	int matrix_height(const VW::ImageBase<T,U>& A) {
		return A.GetHeight();
	}
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
		static const int fixed_rows = Rows;
		static const int fixed_cols = Cols;
		static const bool is_fixed_size = true;
	};

	template <class Precision, class Layout>
	struct matrix_traits<toon::Matrix<-1,-1,Precision,Layout> > {
		typedef typename toon::Matrix<-1,-1,Precision,Layout> matrix_type;
		typedef Precision value_type;
		typedef Layout layout;
		static const bool is_fixed_size = false;
	};

	template <class Precision>
	struct matrix_traits<VNL::Matrix<Precision> > {
		typedef typename VNL::Matrix<Precision> matrix_type;
		typedef Precision value_type;
		static const bool is_fixed_size = false;
	};

}  // namespace indoor_context
