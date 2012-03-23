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

	// Compute the digamma function (gamma function divided by its derivative)
	float FastDigamma (float x);

	// Compute the following sum involving a harmonic series
	//    sum_{x=x0}^{x1} 1. / (a*x + b)
	//
	// The sum bounds are inclusive, the result is a very close approximation of:
	//    float hsum = 0.;
	//    for (int x = x0; x <= x1; x++) {
	//      hsum += 1. / (a*x+b);
	//    }
	float FastHarmonicSum(float a, float b, float x0, float x1);
}  // namespace indoor_context
