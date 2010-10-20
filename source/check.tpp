/*
 * check.tpp
 *
 *  Created on: 6 Aug 2010
 *      Author: alexf
 */
#pragma once

#include <exception>

#include "common_types.h"
#include "log.tpp"

// Turn a C++ expression into "expr: <value-of-expr>\n"
#define EXPR_STR(x) AssertionManager::GetExprStr(#x,x)

// The basic assertion
//    Usage: CHECK(myVariable.someBooleanFunc(foo)) << "the function failed for args " << foo;
#define CHECK(x)															\
		if (!(x))															\
			if (DelayedError __delayed_error = -1)							\
				__delayed_error <<											\
					"Check failed: '" #x "'\n  at " __FILE__ ":" << __LINE__ <<endl

// Check that an arbitrary binary operation evaluates to true
#define CHECK_BINOP(x,op,y)												\
		CHECK((x) op (y)) << EXPR_STR(x) << EXPR_STR(y)

// Basic checks
#define CHECK_EQ(x,y) CHECK_BINOP(x,==,y)
#define CHECK_NE(x,y) CHECK_BINOP(x,!=,y)
#define CHECK_LT(x,y) CHECK_BINOP(x,<,y)
#define CHECK_LE(x,y) CHECK_BINOP(x,<=,y)
#define CHECK_GT(x,y) CHECK_BINOP(x,>,y)
#define CHECK_GE(x,y) CHECK_BINOP(x,>=,y)
#define CHECK_NULL(x) CHECK_EQ(x,NULL)
#define CHECK_NOT_NULL(x) CHECK_NE(x,NULL)

// Check that |x-y| < eps
#define CHECK_EQ_TOL(x,y,eps) \
		CHECK(abs((x)-(y))<=eps) << EXPR_STR(x) << EXPR_STR(y)

// Check that f(x) returns true. Will report the value of x on failure
#define CHECK_PRED1(f,x)												\
		CHECK(f(x)) << EXPR_STR(x)

// Check that f(x,y) returns true. Will report x and y on failure
#define CHECK_PRED2(f,x,y)												\
		CHECK(f(x,y)) << EXPR_STR(x) << EXPR_STR(y)

// Same as CHECK_PRED1
#define CHECK_PRED(f,x) CHECK_PRED1(f,x)

// Check that i is in the closed interval [a,b]
#define CHECK_INTERVAL(i,a,b) CHECK((i) >= (a) && (i) <= (b))	\
		<< EXPR_STR(i) << EXPR_STR(a) << EXPR_STR(b)

// Check that i is a valid index into the specified range.
//    vector<int> x(numberOfItems);
//    int i;
//    cin >> i;
// Any container compatible with Boost.Range can be used.
#define CHECK_INDEX(i,range) CHECK_INTERVAL(i, 0, size(range));


namespace indoor_context {

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
		kErrorModeCurrent = 0,  // Not an error mode: used to get the current error mode
		kErrorModeThrow = 1,
		kErrorModeExit = 2
	};

	// Get/set the current error mode
	static int ErrorMode(ErrorModes mode=kErrorModeCurrent) {
		static int cur_mode = kErrorModeExit;  // default to exiting on assertion failure
		if (mode != kErrorModeCurrent) {
			cur_mode = mode;
		}
		return cur_mode;
	}

	// CHECK* calls will throw exceptions on failure
	static void SetExceptionMode() {
		ErrorMode(kErrorModeThrow);
	}

	// CHECK* calls will exit on failure
	static void SetExitMode() {
		ErrorMode(kErrorModeExit);
	}

	// Simple heuristic to guess whether a given string represents a
	// literal C++ expression
	static inline bool IsLiteral(const string& expr) {
		return expr[0] == '"' || expr[0] == '\'' || expr[0] == '-' || isdigit(expr[0]);
	}

	// Report a failed check
	template <typename T>
	static string GetExprStr(const string& expr, const T& val) {
		// Constructing a format object is expensive so avoid constructing
		// one on each invokation.
		if (IsLiteral(expr)) {
			return "";
		} else {
			// Use stringstream here rather than boost::format to avoid long compilation
			stringstream ss;
			ss << "  " << expr << " = " << val << endl;
			return ss.str();
		}
	}
};

// Represents an object that either exits or throws an exception on destruction
class DelayedError {
	int exitval;
	shared_ptr<stringstream> ss;
public:
	DelayedError(const int v) : exitval(v), ss(new stringstream) { }
	virtual ~DelayedError() {
		if (AssertionManager::ErrorMode() == AssertionManager::kErrorModeThrow) {
			// It is bad practice (though perfectly legal C++) to throw exceptions
			// from destructors. However, it's the only way to accomplish
			// the syntax that looks like:
			//     CHECK(foo) << "some extra information";
			// DelayedError is never used except from CHECK() macros,
			// guaranteeing that it will never be in a std::vector or similar
			// situation in which it might cause memory leaks. Of course, CHECK(...)
			// should never appear in another object's destructor.
			throw AssertionFailedException(ss->str());
		} else {
			cerr << ss->str() << endl;
			exit(-1);
		}
	}

	// Allow this object to appear in an IF block
	inline operator bool() const { return true; }

	// Stream things to the child stringstream
	template <typename T>
	ostream& operator<<(const T& x) {
		return (*ss) << x;
	}
};

} // namespace indoor_context
