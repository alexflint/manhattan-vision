#include "common_types.h"
#include "vararg_utils.h"

namespace indoor_context {
	vector<string>* ParseVarargExprs(const string& s) {
		// We are essentially parsing C++ expressions here - uh oh!  We
		// use the following heuristic: commas can only appear in function
		// calls (or in for loops but they do not qualify as rvalues so
		// can't be passed in here), and they must be surrounded by
		// parentheses, so we should only need to track parentheses, not
		// other types of brackets.

		vector<string>* vs = new vector<string>;
		int a = 0, depth = 0;
		for (int i = 0; i <= s.length(); i++) {
			if (i == s.length() || (depth == 0 && s[i] == ',')) {
				vs->push_back(s.substr(a, i-a));
				a = i+1;
				while (a < s.length() && s[a] == ' ') a++;
			} else if (s[i] == '(') {
				depth++;
			} else if (s[i] == ')') {
				depth--;
			}
		}
		return vs;
	}
}
