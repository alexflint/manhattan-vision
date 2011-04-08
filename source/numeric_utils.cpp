/*
 * math_utils.cpp
 *
 *  Created on: 2 Jun 2010
 *      Author: alexf
 */

#include "numeric_utils.h"
#include <boost/foreach.hpp>
#include "common_types.h"

namespace indoor_context {
	double LogSumExp(double y1, double y2) {
		double m = max(y1, y2);
		return log(exp(y1-m) + exp(y2-m)) + m;
	}

	double LogSumExp(const VecD& ys) {
		// Here we scale and then unscale by a constant m to avoid machine
		// precision issues.
		double sum = 0.0, m = ys.MaxValue();
		BOOST_FOREACH(const double& y, ys) {
			sum += exp(y - m);
		}
		return log(sum) + m;
	}

	double LogSumExp(const double* ys, int n) {
		// Here we scale and then unscale by a constant m to avoid machine
		// precision issues.
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

}
