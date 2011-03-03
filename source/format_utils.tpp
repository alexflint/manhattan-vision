#include <boost/format.hpp>

#include "common_types.h"

namespace indoor_context {
	template <typename T>
	string fmt(const string& s, const T& x) {
		return boost::str(boost::format(s) % x);
	}

	template <typename T1, typename T2>
	string fmt(const string& s, const T1& x1, const T2& x2) {
		return boost::str(boost::format(s) % x1 % x2);
	}

	template <typename T1, typename T2, typename T3>
	string fmt(const string& s, const T1& x1, const T2& x2, const T3& x3) {
		return boost::str(boost::format(s) % x1 % x2 % x3);
	}
}
