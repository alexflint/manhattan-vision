#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include "common_types.h"
#include "numeric_utils.h"
#include "likelihoods.h"

#include "protobuf_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;
	using boost::ref;
	using boost::bind;
	using boost::function;

	double EvaluateModelLogLik(ModelLikelihood& likelihood,
														 const proto::PayoffFeatureSet& featureset,
														 const Vec2& lambda) {
		likelihood.Configure(lambda);
		BOOST_FOREACH(const proto::FrameWithFeatures& instance, featureset.frames()) {
			likelihood.Process(instance);
		}
		return likelihood.log_likelihood();
	}

	double EvaluateFeatureLogLik(FeatureLikelihood& likelihood,
															 const proto::PayoffFeatureSet& featureset,
															 const Vector<>& theta) {
		likelihood.Configure(theta);
		BOOST_FOREACH(const proto::FrameWithFeatures& instance, featureset.frames()) {
			likelihood.Process(instance);
		}
		return likelihood.log_likelihood();
	}

	double EvaluateLikelihood(const ManhattanHyperParameters& params,
														const string& features_file,
														bool logit_likelihood=false) {
		// Load Features
		CHECK_PRED1(fs::exists, features_file);
		proto::PayoffFeatureSet feature_set;
		ReadLargeProto(features_file, feature_set);

		CHECK_GE(params.corner_penalty, 1e-18)
			<< "Penalties mut be positive (since constant in geometric series must be < 1)";
		CHECK_GE(params.occlusion_penalty, 1e-18)
			<< "Penalties mut be positive (since constant in geometric series must be < 1)";

		// Initialize likelihood functions
		Vec2 lambda = makeVector(params.corner_penalty, params.occlusion_penalty);
		Vector<> theta = asToon(params.weights);
		scoped_ptr<ModelLikelihood> model_lik(new ModelLikelihood(lambda));
		scoped_ptr<FeatureLikelihood> ftr_lik;
		if (logit_likelihood) {
			DLOG << "Using logistic feature likelihood";
			ftr_lik.reset(new LogitFeatureLikelihood(theta));
		} else {
			DLOG << "Using Gaussian feature likelihood";
			ftr_lik.reset(new GaussianFeatureLikelihood(theta, false));
		}

		// Prepare functions (for convenience with numerical gradients)
		function<double(const Vec2&)> model_loglik_func =
			bind(&EvaluateModelLogLik, ref(*model_lik), ref(feature_set), _1);
		function<double(const Vector<>&)> ftr_loglik_func =
			bind(&EvaluateFeatureLogLik, ref(*ftr_lik), ref(feature_set), _1);

		// Compute likelihood
		double model_loglik = model_loglik_func(lambda);
		double ftr_loglik = ftr_loglik_func(theta);
		return ftr_loglik + model_loglik;	
	}
}
