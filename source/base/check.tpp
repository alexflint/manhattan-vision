/*
 * check.tpp
 *
 *  Created on: 6 Aug 2010
 *      Author: alexf
 */
#pragma once

#include <exception>

#include <boost/shared_ptr.hpp>

#include "common_types.h"
#include "log.tpp"
#include "matrix_traits.tpp"  // for CHECK_SAME_SIZE etc, generates no extra includes

// The basic assertion
//    Usage: CHECK(myVariable.someBooleanFunc(foo)) << "the function failed for args " << foo;
#define CHECK(x)																												\
	if (!(x))																															\
		if (NS::DelayedError __delayed_error = -1)													\
			__delayed_error <<																								\
				"Check failed: '" #x "'\n  at " __FILE__ ":" << __LINE__ << std::endl

// Check that an arbitrary binary operation evaluates to true
#define CHECK_BINOP(x,op,y)											\
	CHECK((x) op (y)) << EXPR(x,y)

// Basic checks
#define CHECK_EQ(x,y) CHECK_BINOP(x,==,y)
#define CHECK_NE(x,y) CHECK_BINOP(x,!=,y)
#define CHECK_LT(x,y) CHECK_BINOP(x,<,y)
#define CHECK_LE(x,y) CHECK_BINOP(x,<=,y)
#define CHECK_GT(x,y) CHECK_BINOP(x,>,y)
#define CHECK_GE(x,y) CHECK_BINOP(x,>=,y)
#define CHECK_NULL(x) CHECK_EQ(x,NULL)
#define CHECK_NOT_NULL(x) CHECK(x != NULL)

// Check that |x-y| < eps
#define CHECK_EQ_TOL(x,y,eps)										\
	CHECK(err(x,y)<=eps) << EXPR(x,y)

// Check that f(x) returns true. Will report the value of x on failure
#define CHECK_PRED1(f,x)												\
	CHECK(f(x)) << EXPR(x)

// Check that f(x,y) returns true. Will report x and y on failure
#define CHECK_PRED2(f,x,y)											\
	CHECK(f(x,y)) << EXPR(x,y)

// Same as CHECK_PRED1
#define CHECK_PRED(f,x) CHECK_PRED1(f,x)

// Check that i is in the closed interval [a,b]
#define CHECK_INTERVAL(i,a,b)										\
	CHECK((i) >= (a) && (i) <= (b)) << EXPR(i,a,b)

// Check that i is a valid index into the specified range.
// Any container compatible with Boost.Range can be used.
#define CHECK_INDEX(i,range)										\
	CHECK_INTERVAL(i, 0, size(range)-1);

// Check that two objects are the same size. Uses matrix_size() from matrix_utils.tpp
#define CHECK_SAME_SIZE(a,b)												\
	CHECK_EQ(NS::matrix_size(a), NS::matrix_size(b))

// Check that a 2D vector is within the bounds of a matrix
#define CHECK_POS(pos, m)														\
	CHECK(pos[0] >= 0 && pos[0] < matrix_width(m) &&	\
				pos[1] >= 0 && pos[1] < matrix_height(m))		\
	<< EXPR(pos, matrix_size(m))

namespace indoor_context {
	// Distance computation for CHECK
	template <typename T, typename U>
	double err(const T& a, const U& b) {
		return abs(a-b);
	}

	// The exception to throw when an assertion fails (and we're in kErrorModeThrow mode)
	class AssertionFailedException : public std::exception {
	public:
		string err;
		AssertionFailedException(const string& s) : err(s) { }
		virtual ~AssertionFailedException() throw() { }
		virtual const char* what() const throw() { return err.c_str(); }
	};

	// Manages assertions
	class AssertionManager {
	public:
		// Error modes for assertion failures
		enum ErrorModes {
			kErrorModeCurrent = 0,  // Used to get the current error mode
			kErrorModeThrow = 1,
			kErrorModeExit = 2
		};

		// Get/set the current error mode
		static int ErrorMode(ErrorModes mode=kErrorModeCurrent);
		// CHECK* calls will throw exceptions on failure
		static void SetExceptionMode() { ErrorMode(kErrorModeThrow); }
		// CHECK* calls will exit on failure
		static void SetExitMode() { ErrorMode(kErrorModeExit); }
	};

	// Represents an object that either exits or throws an exception on destruction
	class DelayedError {
		int exitval;
		boost::shared_ptr<stringstream> ss;
	public:
		DelayedError(const int v) : exitval(v), ss(new stringstream) { }
		// Throws an exception or calls exit(-1), depending on AssertionManager::ErrorMode().
		virtual ~DelayedError();
		// Allow this object to appear in an IF block
		inline operator bool() const { return true; }

		// Stream things to the child stringstream
		template <typename T>
		ostream& operator<<(const T& x) {
			return (*ss) << x;
		}
	};
} // namespace indoor_context
