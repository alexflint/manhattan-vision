// This file provides a logger with two key functions:
//
// 1. Lines will be ended automatically, so we can do DLOG << "foo";
// and a newline will be inserted. However, if we do DLOG << "foo\n"
// then we will only get one newline (the explicit newline will
// override the automatic one).
//
// 2. We can set up indentation by using the INDENTED{...} block,
// which causes all output in that block to be indented.


// There are several subtleties in automatically ending lines. The
// DLOG macro creates an ScopedTokenSender object, which, upon
// destruction, sends a special token to the stream. When the stream
// recieves this it prints a newline character unless a newline
// character immediately preceeded it (i.e. lines that have already
// been ended will not be ended again since that would leave an empty
// line).
//
// Streams contain internal buffering so it is important to
// communicate by sending tokens rather than by direct method calls
// (which might will out of sync with the stream if the buffer is
// partially filled).
//
// We also flush the stream after the end of each line. This is
// slightly inefficient but very useful for logging purposes.

#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include "common_types.h"

// Here we use an if block to create a scope that does not need an
// ending brace (so that the code looks nice). The assignment is used
// to initialize the scoped_line_ender, which always returns true
// because ScopedTokenSender has an overload for "operator bool".

// The namespace in which the log machinery is defined
#define NS ::indoor_context

// The underying log stream
#define DLOG_STREAM NS::GetLogInstance()

// Log to output
#define DLOG																			\
		if (NS::LogState::enabled)											\
		if (NS::ScopedTokenSender __x = DLOG_STREAM)	\
		DLOG_STREAM

// Log to output, don't append a newline
#define DLOG_N																	\
		if (NS::LogState::enabled) DLOG_STREAM

// Enable logging in current scope
#define ENABLE_DLOG																								\
		NS::LogState::ScopedEnabler __enabler__ ## __LINE__ ## _(true)

// Disable logging in current scope
#define DISABLE_DLOG																							\
		NS::LogState::ScopedEnabler __disabler__ ## __LINE__ ## _(false)

// Enable logging in current scope
#define WITH_DLOG																												\
		if (NS::LogState::ScopedEnabler __enabler__ ## __LINE__ ## _ = true)

// Disable logging in current scope
#define WITHOUT_DLOG																										\
		if (NS::LogState::ScopedEnabler __disabler__ ## __LINE__ ## _ = false)

// Increase indent in current scope
#define SCOPED_INDENT																						\
		NS::LogState::ScopedIndenter __indenter__ ## __LINE__ ## _(0)

// Create a block where logging is increased
#define INDENTED																									\
		if (NS::LogState::ScopedIndenter __indenter__ ## __LINE__ = 0)

// Increase indent in current scope, print a newline at the end of the
// current scope
#define SPACED_INDENT																						\
		NS::LogState::ScopedIndenter __indenter__ ## __LINE__ ## _(1)

// Write a string then increase the title
#define TITLE(title) DLOG << title; SCOPED_INDENT;

// Write a string then create a scope in which indentation is increased
#define TITLED(title) DLOG << title; INDENTED

// These tell DREPORT how to print without newlines being appended
#define cout_N cout
#define cerr_N cerr

// Report a variable name and value
#define DREPORT_S(x, s)																		\
		{																												\
	s ## _N << #x << ": ";																\
	int __xlen = strlen(#x)+2;														\
	NS::LogState::IncreaseIndent(__xlen);									\
	s << (x) << endl;																			\
	NS::LogState::DecreaseIndent(__xlen);									\
		}

// Report a variable name and its value
#define DREPORT(...) NS::ReportExprs(__VA_ARGS__ , NS::ParseVaExprs(#__VA_ARGS__));


// Assertions
#define CHECK(x)																								\
		if (!(x))																											\
		if (Quiter __q = -1)																				\
		WITH_DLOG DLOG << "Check failed: '"<<#x<<"'\n"						\
		<< "  at "<<__FILE__<<":"<<__LINE__<<endl

#define CHECK_GENERIC(x,op,y)													\
		CHECK(x op y) << GetExprStr(#x, x) << GetExprStr(#y, y)

#define CHECK_EQ(x,y) CHECK_GENERIC(x,==,y)
#define CHECK_NE(x,y) CHECK_GENERIC(x,!=,y)
#define CHECK_LT(x,y) CHECK_GENERIC(x,<,y)
#define CHECK_LE(x,y) CHECK_GENERIC(x,<=,y)
#define CHECK_GT(x,y) CHECK_GENERIC(x,>,y)
#define CHECK_GE(x,y) CHECK_GENERIC(x,>=,y)
#define CHECK_NULL(x) CHECK_EQ(x,NULL)
#define CHECK_NOT_NULL(x) CHECK_NE(x,NULL)

#define CHECK_TOL(x,y,eps) \
		CHECK(abs(x-y)<eps) << GetExprStr(#x,x) << GetExprStr(#y,y)

#define CHECK_PRED1(f,x)												\
		CHECK(f(x)) << GetExprStr(#x,x)
#define CHECK_PRED2(f,x,y)																	\
		CHECK(f(x,y)) << GetExprStr(#x,x) << GetExprStr(#y,y)

#define CHECK_PRED(f,x) CHECK_PRED1(f,x)

#define CHECK_INTERVAL(i,a,b) CHECK(i >= a && i <= b)	\
		<< GetExprStr(#i,i)																	\
		<< GetExprStr(#a,a)																	\
		<< GetExprStr(#b,b)
#define CHECK_INDEX(i,range) CHECK_INTERVAL(i, 0, size(range));

namespace indoor_context {

// Get the singleton log stream
//boost::iostreams::filtering_ostream& GetLogInstance();
std::ostream& GetLogInstance();

struct SpecialLogTokens {
	static const int kNullToken = -1;
	static const int kEndToken = -2;
	static const int kNewline = '\n';
};

// Sends AutoNewlineFilter::kEndToken to a specified stream on
// destruction
class ScopedTokenSender {
public:
	ostream& str;
	inline ScopedTokenSender(ostream& s) : str(s) { }
	// Send an "end this line now" token followed by a flush token
	inline ~ScopedTokenSender() {
		str.put(SpecialLogTokens::kEndToken);
		str << flush;
	}
	// So that we can be used inside an if block for the DLOG macro
	inline operator bool() {
		return true;
	}
};

// Represents global log state
class LogState {
private:
public:
	static bool enabled;
	static int indent_level;  // Global indent level

	// Control the indent level
	static void SetIndent(int new_level);

	// Increase/decrease using string length
	static inline void IncreaseIndent(int n) {
		SetIndent(indent_level+n);
	}
	static inline void DecreaseIndent(int n) {
		SetIndent(indent_level-n);
	}

	static inline void Enable() {
		enabled = true;
	}
	static inline void Disable() {
		enabled = false;
	}

	// A ScopedIndenter increases the global indent level in its
	// constructor and decreases it by the same amount in its
	// destructor. To have the log output of a code block indented use
	// the INDENT macro to declare a stack-allocated ScopedIndenter
	// instance, eg:
	// for (..something..) {
	//   INDENT;
	//   some_other_stuff();
	//   DLOG << "the value of x is " << x << endl;
	//   ...
	// }
	class ScopedIndenter {
	private:
		int nlines;
	public:
		static const int kIndentIncrement = 2;
		ScopedIndenter(int lines = 0);
		~ScopedIndenter();
		inline operator bool() { return true; }
	};

	// A ScopedEnabler changes the enabled/disabled state of log output in
	// its constructor and restores the old state in its destructor. It
	// can be used to control which blocks produce debug output.
	class ScopedEnabler {
	private:
		bool oldstate;
	public:
		ScopedEnabler(bool newstate);
		~ScopedEnabler();
		inline operator bool() { return true; }
	};
};

// Represents a class that calls exit(s) on destruction
struct Quiter {
	const int ret;
	inline Quiter(const int r) : ret(r) { }
	inline ~Quiter() {
		cerr << "Exiting" << endl;
		exit(ret);
	}
	inline operator bool() { return true; }
};

// Simple heuristic to guess whether a given string represents a
// literal C++ expression
inline bool IsLiteral(const string& expr) {
	return !isalpha(expr[0]) && expr[0] != '_';
}

// Report a failed check
template <typename T>
string GetExprStr(const string& expr, const T& val) {
	// Constructing a format object is expensive so avoid constructing
	// one on each invokation.
	if (IsLiteral(expr)) {
		return "";
	} else {
		// Use stringstream here rather than boost::format to avoid long compilation
		stringstream ss;
		ss << expr << " = " << val;
		return ss.str();
	}
}



// Parse a set of expressions of the form "EXPR1, EXPR2, "...x
vector<string> ParseVaExprs(const string& s);

// Report an expression and its value
template <typename T>
void ReportExpr(const T& val, const string& expr) {
	DLOG_N << expr << ": ";
	LogState::IncreaseIndent(expr.length()+2);
	DLOG << val << endl;
	LogState::DecreaseIndent(expr.length()+2);
}

// Report several expressions
template <typename A>
void ReportExprs(const A& a, const vector<string>& exprs) {
	ReportExpr(a, exprs[0]);
}

// Report several expressions
template <typename A, typename B>
void ReportExprs(const A& a,
                 const B& b,
                 const vector<string>& exprs) {
	ReportExpr(a, exprs[0]);
	ReportExpr(b, exprs[1]);
}

// Report several expressions
template <typename A, typename B, typename C>
void ReportExprs(const A& a,
                 const B& b,
                 const C& c,
                 const vector<string>& exprs) {
	ReportExpr(a, exprs[0]);
	ReportExpr(b, exprs[1]);
	ReportExpr(c, exprs[2]);
}

// Report several expressions
template <typename A, typename B, typename C, typename D>
void ReportExprs(const A& a,
                 const B& b,
                 const C& c,
                 const D& d,
                 const vector<string>& exprs) {
	ReportExpr(a, exprs[0]);
	ReportExpr(b, exprs[1]);
	ReportExpr(c, exprs[2]);
	ReportExpr(d, exprs[3]);
}

// Report several expressions
template <typename A, typename B, typename C, typename D, typename E>
void ReportExprs(const A& a,
                 const B& b,
                 const C& c,
                 const D& d,
                 const E& e,
                 const vector<string>& exprs) {
	ReportExpr(a, exprs[0]);
	ReportExpr(b, exprs[1]);
	ReportExpr(c, exprs[2]);
	ReportExpr(d, exprs[3]);
	ReportExpr(e, exprs[4]);
}

// Report several expressions
template <typename A, typename B, typename C, typename D, typename E, typename F>
void ReportExprs(const A& a,
                 const B& b,
                 const C& c,
                 const D& d,
                 const E& e,
                 const F& f,
                 const vector<string>& exprs) {
	ReportExpr(a, exprs[0]);
	ReportExpr(b, exprs[1]);
	ReportExpr(c, exprs[2]);
	ReportExpr(d, exprs[3]);
	ReportExpr(e, exprs[4]);
	ReportExpr(f, exprs[5]);
}

// Report several expressions
template <typename A, typename B, typename C, typename D, typename E, typename F, typename G>
void ReportExprs(const A& a,
                 const B& b,
                 const C& c,
                 const D& d,
                 const E& e,
                 const F& f,
                 const G& g,
                 const vector<string>& exprs) {
	ReportExpr(a, exprs[0]);
	ReportExpr(b, exprs[1]);
	ReportExpr(c, exprs[2]);
	ReportExpr(d, exprs[3]);
	ReportExpr(e, exprs[4]);
	ReportExpr(f, exprs[5]);
	ReportExpr(g, exprs[6]);
}


// Report several expressions
template <typename A, typename B, typename C, typename D,
typename E, typename F, typename G, typename H>
void ReportExprs(const A& a,
                 const B& b,
                 const C& c,
                 const D& d,
                 const E& e,
                 const F& f,
                 const G& g,
                 const H& h,
                 const vector<string>& exprs) {
	ReportExpr(a, exprs[0]);
	ReportExpr(b, exprs[1]);
	ReportExpr(c, exprs[2]);
	ReportExpr(d, exprs[3]);
	ReportExpr(e, exprs[4]);
	ReportExpr(f, exprs[5]);
	ReportExpr(g, exprs[6]);
	ReportExpr(h, exprs[7]);
}
}
