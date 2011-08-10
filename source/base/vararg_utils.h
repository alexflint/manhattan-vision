#include "common_types.h"

// This is in a file all of its own because it is used by log.tpp and
// report.tpp, which need to have absolutely minimal dependencies.
namespace indoor_context {
	// Parse a set of expressions of the form "EXPR1, EXPR2, "...x
	vector<string>* ParseVarargExprs(const string& s);
}
