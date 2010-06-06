/*
 * math_utils.cpp
 *
 *  Created on: 2 Jun 2010
 *      Author: alexf
 */

#include <sstream>
#include <iomanip>

#include <boost/foreach.hpp>

#include "common_types.h"
#include "math_utils.h"

namespace indoor_context {

double LogSumExp(const VecD& ys) {
	// Here we scale and then unscale by a factor k to avoid machine
	// precision issues. Mathematically the result is identical.
	double sum = 0.0, m = ys.MaxValue();
	BOOST_FOREACH(const double& y, ys) {
		sum += exp(y - m);
	}
	return log(sum) + m;
}

double LogSumExp(const double* ys, int n) {
	// Here we scale and then unscale by a factor k to avoid machine
	// precision issues. Mathematically the result is identical.
	double sum = 0.0, maxy = -INFINITY;
	for (int i = 0; i < n; i++) {
		if (ys[i] > maxy) maxy = ys[i];
	}
	for (int i = 0; i < n; i++) {
		sum += exp(ys[i] - maxy);
	}
	return log(sum) + maxy;
}

void NormalizeLogDistr(VecD& ys) {
	ys -= LogSumExp(ys);
}

VecD LogLikelihoodToDistr(const VecD& ys) {
	VecD distr = ys;
	NormalizeLogDistr(distr);
	return distr.Apply(exp);
}

string PaddedInt(int x, int width) {
	stringstream ss;
	ss << setfill('0') << setw(width) << x;
	return ss.str();
}

}
