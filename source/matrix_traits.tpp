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
