/*
 * math_utils.h
 *
 *  Created on: 2 Jun 2010
 *      Author: alexf
 */

#pragma once

#include "common_types.h"

namespace indoor_context {
	// Note that here the variance is the entire "sigma squared" term in
	// the Normal distribution (so variance = sigma*sigma)

	// Evaluate a 1D gaussion centred at m with variance s
	double Gauss1D(double x, double m, double var);
	// Evaluate a 2D gaussion centred at m with variance s
	//double Gauss2D(const Vec2& x, const Vec2& m, double var);
	// Evaluate the log of the 1D gaussian
	double LogGauss1D(double x, double m, double var);
	// Evaluate the log of the 1D gaussian using precomputed values for speed.
 	double FastLogGauss1D(double x, double m, double var, double log_var);

	// Compute log(exp(y1) + exp(y2)) in a numerically stable way
	double LogSumExp(double y1, double y2);
	// Compute log(exp(y1) + exp(y2)) in a numerically stable way
	double LogSumExp(double y1, double y2, double y3);
	// Compute log( exp(y1)+exp(y2)+...+exp(yn) ) in a numerically
	// stable way
	double LogSumExp(const VecD& ys);
	// Compute log( exp(y1)+exp(y2)+...+exp(yn) ) in a numerically
	// stable way
	double LogSumExp(const double* ys, int n);

	// Given a vector (y1, y2, ...), normalize it so that it represents
	// the log of a valid distribution, i.e.:
	//   exp(y1) + exp(y2) + ... + exp(yn) = 1
	// Avoid underflow by using LogSumExp
	void NormalizeLogDistr(VecD& ys);

	// Tranform a vector of log likelihoods into a normalized
	// distribution. This consists of exponentiating the terms and
	// ensuring that they sum to 1. Uses LogSumExp for numerical
	// stability.
	VecD LogLikelihoodToDistr(const VecD& ys);
}  // namespace indoor_context
