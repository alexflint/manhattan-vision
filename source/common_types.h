#pragma once

#include <string>
#include <vector>
#include <utility>
#include <exception>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/lexical_cast.hpp>

#include <VNL/vector.h>
#include <VNL/vectorfixed.h>
#include <VNL/matrix.h>
#include <VNL/matrixfixed.h>

#include <TooN/TooN.h>

#include <gvars3/instances.h>

// forward-declare boost::filesystem in order to introduce namespace alias
#define BOOST_FILESYSTEM_VERSION 2  // TODO: change to 3
namespace boost { namespace filesystem { } }

namespace indoor_context {
	using namespace std;
	using boost::scoped_ptr;
	using boost::scoped_array;
	namespace fs=boost::filesystem;
	namespace toon=TooN;

	typedef unsigned char byte;

	typedef toon::Vector<2> Vec2;
	typedef toon::Vector<3> Vec3;
	typedef toon::Vector<4> Vec4;
	typedef toon::Vector<5> Vec5;
	typedef toon::Vector<6> Vec6;

	typedef toon::Vector<2,int> Vec2I;
	typedef toon::Vector<3,int> Vec3I;
	typedef toon::Vector<4,int> Vec4I;
	typedef toon::Vector<5,int> Vec5I;
	typedef toon::Vector<6,int> Vec6I;

	typedef toon::Matrix<2> Mat2;
	typedef toon::Matrix<3> Mat3;
	typedef toon::Matrix<4> Mat4;

	typedef VNL::Matrix<int> MatI;
	typedef VNL::Matrix<float> MatF;
	typedef VNL::Matrix<double> MatD;

	typedef VNL::Vector<int> VecI;
	typedef VNL::Vector<float> VecF;
	typedef VNL::Vector<double> VecD;

} // indoor_context

// These rely on the above typedefs...
#include "log.tpp"
#include "check.tpp"
#include "lazyvar.h"
#include "initialized_ptr.tpp"
