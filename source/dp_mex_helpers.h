/*
 * dp_mex_helpers.h
 *
 *  Created on: 12 Aug 2010
 *      Author: alexf
 */
#pragma once

#include <boost/format.hpp>

#include "common_types.h"
#include "matlab_utils.h"
#include "map.h"
#include "map.pb.h"

namespace indoor_context {
using namespace toon;

template <typename T>
string sformat(const char* s, const T& x) {
	return str(format(s) % x);
}
template <typename T1, typename T2>
string sformat(const char* s, const T1& x1, const T2& x2) {
	return str(format(s) % x1 % x2);
}
template <typename T1, typename T2, typename T3>
string sformat(const char* s, const T1& x1, const T2& x2, const T3& x3) {
	return str(format(s) % x1 % x2 % x3);
}
template <typename T1, typename T2, typename T3, typename T4>
string sformat(const char* s,
               const T1& x1,
               const T2& x2,
               const T3& x3,
               const T4& x4) {
	return str(format(s) % x1 % x2 % x3 % x4);
}

}  // namespace indoor_context
