#pragma once

#include "common_types.h"
#include "payoff_helpers.h"

namespace indoor_context {
	namespace proto { class FrameWithFeatures; }

	// Computes P(z | lambda)
	class ModelLikelihood {
	public:
		// Initialize
		ModelLikelihood();
		ModelLikelihood(const Vec2& lambda);
		// Reset internal state
		void Configure(const Vec2& lambda);
		// Process a training instance
		void Process(const proto::FrameWithFeatures& instance);
		// Returns the likelihood
		double log_likelihood() const { return loglik; }
		// Returns the jacobian of the *log* likelihood
		const Vec2& jacobian() const { return J_loglik; }

		// Populate the penalty terms of a payoff function
		void PopulatePayoffs(DPPayoffs& payoffs);

	private:
		Vec2 lambda;
		double loglik;
		Vec2 J_loglik;
	};

	// Interface for feature likelihoods
	class FeatureLikelihood {
	public:
		// Configure this likelihood
		virtual void Configure(const toon::Vector<>& params) = 0;
		// Process a training instance
		virtual void Process(const proto::FrameWithFeatures& instance) = 0;
		// Returns the likelihood
		virtual double log_likelihood() const = 0;
		// Returns the jacobian of the *log* likelihood
		virtual const toon::Vector<>& jacobian() const = 0;
	};

	// Computes a logistic likelihood
	class LogitFeatureLikelihood : public FeatureLikelihood {
	public:
		LogitFeatureLikelihood(const toon::Vector<>& theta);
		// Reset internal state
		void Configure(const toon::Vector<>& theta);
		// Process a training instance
		void Process(const proto::FrameWithFeatures& instance);
		// Returns the likelihood
		double log_likelihood() const { return loglik; }
		// Returns the jacobian of the *log* likelihood
		const toon::Vector<>& jacobian() const { return J_loglik; }
	private:
		toon::Vector<> theta;
		double loglik;
		toon::Vector<> J_loglik;
		MatF path_ftrs;  // internal buffer only
	};


	// Computes a mixture-of-Gaussians likelihood
	class GaussianFeatureLikelihood : public FeatureLikelihood {
	public:
		GaussianFeatureLikelihood(const toon::Vector<>& theta,
															bool enable_jacobian=true);
		// Reset internal state
		void Configure(const toon::Vector<>& theta);
		// Process a training instance
		void Process(const proto::FrameWithFeatures& instance);
		// Returns the likelihood
		double log_likelihood() const { return loglik; }
		// Returns the jacobian of the *log* likelihood
		const toon::Vector<>& jacobian() const { return J_loglik; }
		// whether to compute jacobians
		bool enable_jacobian;

		// Compute payoffs
		void ComputePayoffs(const proto::FrameWithFeatures& instance,
												DPPayoffs& payoffs) const;
	private:
		toon::Vector<> theta;
		double loglik;
		toon::Vector<> J_loglik;
		mutable PayoffFeatures features;  // internal buffer only
	};
}
