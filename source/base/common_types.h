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

#include <gvars3/instances.h>

// forward-declare boost::filesystem in order to introduce namespace alias
namespace boost { namespace filesystem { } }

namespace indoor_context {
	using namespace std;
	using boost::scoped_ptr;
	using boost::scoped_array;
	namespace fs=boost::filesystem;
	namespace toon=TooN;

	typedef unsigned char byte;
} // indoor_context

// These rely on the above typedefs...
#include "matrix_types.h"
#include "log.tpp"    // defines DLOG and friends
#include "report.tpp" // defines DREPORT and friends
#include "check.tpp"  // defines CHECK and friends
#include "lazyvar.h"
