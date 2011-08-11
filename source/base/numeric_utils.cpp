#include "numeric_utils.h"

#include <boost/foreach.hpp>

#include "common_types.h"

namespace indoor_context {
	// Constants for efficiency
	static const double kSqrtTwoPi = sqrt(2*M_PI);
	static const double kLogSqrtTwoPi = log(sqrt(2*M_PI));

	double Gauss1D(double x, double m, double var) {
		return exp(-(x-m)*(x-m)/(2*var)) / (sqrt(var)*kSqrtTwoPi);
	}

	/*double Gauss2D(const Vec2& x, const Vec2& m, double s) {
		return exp(-0.5 * norm_sq(m-x) / s) / (2*M_PI*sqrt(s));
		}*/

	double LogGauss1D(double x, double m, double var) {
		return FastLogGauss1D(x, m, var, log(var));
	}

	double FastLogGauss1D(double x, double m, double var, double log_var) {
		return -(x-m)*(x-m)/(2*var) - log_var/2 - kLogSqrtTwoPi;
	}

	double LogSumExp(double y1, double y2) {
		double m = max(y1, y2);
		return log(exp(y1-m) + exp(y2-m)) + m;
	}

	double LogSumExp(double y1, double y2, double y3) {
		double m = max(y1, max(y2, y3));
		return log(exp(y1-m) + exp(y2-m) + exp(y3-m)) + m;
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
