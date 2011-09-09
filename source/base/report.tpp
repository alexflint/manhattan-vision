#pragma once

#include "common_types.h"
#include "streamable.tpp"

// Construct a report for a series of expressions. For example:
//   DLOG << EXPR(foo, bar.getMember(), 1+2+3, myString.firstOccurenceOf("baz"));
#define EXPR(...)																			\
	NS::Report(NS::ParseVarargExprs(#__VA_ARGS__),			\
						 NS::Streamable::NewVector(__VA_ARGS__))

// Report expressions and their values
#define DREPORT(...)									      					\
	DLOG << EXPR(__VA_ARGS__)

namespace indoor_context {
	// Represents a series of expression/value pairs.
	class Report {
	public:
		scoped_ptr<vector<string> > exprs_;
		scoped_ptr<Streamable::vector> vals_;
		Report(vector<string>* exprs, Streamable::vector* vals);
	};

	// Stream insertion for reports
	ostream& operator<<(ostream& o, const Report& report);

	// Parse a set of expressions of the form "EXPR1, EXPR2, ..."
	vector<string>* ParseVarargExprs(const string& s);
}
