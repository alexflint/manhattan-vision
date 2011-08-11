#pragma once

#include <TooN/so3.h>
#include "common_types.h"

namespace indoor_context {
	// Computes a rotation R that minimizes sum(x_i^T * R * y_i).
	// R is guaranteed to be a pure rotation.
	// Minimization is by gradient descent.
	class RotationEstimator {
	public:
		int max_steps;  // initialized from gvar, can be changed after construction
		vector<double> residuals; // residual from all steps
		toon::SO3<> R;  // Current rotation estimate
		Vec3 Jf;  // Jacobian of cost function
		double residual;  // residual
		int num_steps;  // Number of steps since Reset()
		bool converged;  // Whether we've converged

		// Constructor: reads some gvars
		RotationEstimator();
		// Estimate a rotation for the given data
		toon::SO3<>& Compute(const vector<Vec3>& xs,
												 const toon::Matrix<>& responsibilities,
												 const toon::SO3<>& initial_guess);
		// Reset the gradient descent to the identity
		void Reset();
		// Reset the specified start point
		void Reset(const toon::SO3<>& Rinit);
		// Perform one step of the gradient descent
		void Step(const toon::Matrix<>& X,
							const toon::Matrix<>& responsibilities);
	};
}
