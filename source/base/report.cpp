#include "common_types.h"

#include "report.tpp"
#include "streamable.tpp"
#include "log.tpp"

namespace indoor_context {
	Report::Report(vector<string>* exprs, Streamable::vector* vals)
		: exprs_(exprs), vals_(vals) {
		CHECK_EQ(exprs->size(), vals->Size())
			<< "Inside the logging framework!";
	}

	ostream& operator<<(ostream& o, const Report& report) {
		bool islog = (o == LogManager::GetLogStream());
		for (int i = 0; i < report.exprs_->size(); i++) {
			const string& expr = report.exprs_->at(i);
			o << expr << ": ";
			if (islog) {
				LogManager::IncreaseIndent(expr.size()+2);
			}
			o << report.vals_->Get(i);
			if (islog) {
				LogManager::DecreaseIndent(expr.size()+2);
				LogManager::EndCurrentLine();
			} else {
				o << "\n";
			}
		}
		return o;
	}

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
