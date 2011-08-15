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
}
