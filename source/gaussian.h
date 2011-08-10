#pragma once

#include <boost/shared_ptr.hpp>

#include <VNL/Algo/cholesky.h>

#include "common_types.h"

namespace indoor_context {
using boost::shared_ptr;
	// See also simple 1D special cases in numeric_utils.h

	// Represents a gaussian function in arbitrary dimension
	class Gaussian {
	public:
		int dim;  // Dimensionality of the gaussian
		VecD mean;  // Mean
		MatD cov;  // Covariance

		// Initialize a gaussian of dimension D
		Gaussian(int d) : dim(d), mean(d), cov(d, d) { }
		// Evaluate this gaussian at x.
		double Evaluate(const VecD& x) const;
		// Evaluate this gaussian at x.
		double EvaluateLog(const VecD& x) const;
		// Sample from this gaussian
		VecD Sample() const;
	};

	// Represents an object that caches some computations in order to
	// quickly evaluate a gaussian.
	class GaussianEvaluator {
	private:
		const Gaussian& gaussian;
		VNL::Cholesky<double> chol;
		double logdet;
		double denom;
		double logdenom;
	public:
		// Initialize an evaluator for g
		GaussianEvaluator(const Gaussian& g);
		// Evaluate the log of this gaussian at x
		double EvaluateLog(const VecD& x) const;
		// Evaluate this gaussian at x
		double Evaluate(const VecD& x) const;
		// Get the log of the determinant of the covariance matrix
		double GetLogDetCov() const { return logdet; }
	};

	// Represents an object that caches some computations in order to
	// quickly sample from a gaussian.
	class GaussianSampler {
	private:
		const Gaussian& gaussian;
		VNL::Cholesky<double> chol;
	public:
		// Initialize a sampler for g
		GaussianSampler(const Gaussian& g);
		// Sample from the gaussian
		VecD Sample() const;
		// Sample from the standard normal distribution
		static double SampleStdNormal();
	};

	// Represents a linear combination of gaussian functions
	class GaussMixtureEvaluator;
	class GaussMixture {
	public:
		int ncomps;
		int dim;
		VecD weights;
		scoped_array<scoped_ptr<Gaussian> > comps;

		GaussMixture(int ncomps_, int dim_);
		GaussMixtureEvaluator* PrepareEval() const;
		double Evaluate(const VecD& x) const;
		VecD Sample() const;

		void Write(const char* filename) const;
		static GaussMixture* Read(const char* filename);
	};

	// Caches some computation to speed up repeated evaluation of a
	// gaussian mixture.
	class GaussMixtureEvaluator {
	private:
		const GaussMixture& gm;
		vector<shared_ptr<GaussianEvaluator> > evaluators;
	public:
		VecD logweights;
		// Initialize an evaluator for the given model
		GaussMixtureEvaluator(const GaussMixture& model);
		// Read data from the internal reference to the model
		void Prepare();
		// Evaluate this model at x
		double Evaluate(const VecD& x) const;
		// Evaluate this model at x
		double EvaluateLog(const VecD& x) const;
		const GaussianEvaluator& Component(int i) const {
			return *evaluators[i];
		}
	};

	// Caches some computations to speed up repeated sampling from a
	// gaussian mixture
	class GaussMixtureSampler {
	private:
		const GaussMixture& gm;
		double weightsum;
		vector<shared_ptr<GaussianSampler> > samplers;
	public:
		// Initialize a sampler for the given model
		GaussMixtureSampler(const GaussMixture& model);

		// Sample from the model
		VecD Sample() const;
	};

	class GaussMixtureEstimator {
	public:
		int ncomps;
		int max_iters;
		double exit_thresh;
		bool axis_aligned;
		bool spherical;

		GaussMixtureEstimator(int ncomp);
	
		GaussMixture* Estimate(vector<VecD>& pts) const;
	};

	ostream& operator<<(ostream& s, const GaussMixture& model);
}
