#include <exception>
#include <iomanip>

#include <boost/exception/all.hpp>
#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "numeric_utils.h"
#include "likelihoods.h"

#include "io_utils.tpp"
#include "numerical_jacobian.tpp"
#include "protobuf_utils.tpp"

// Window size for finite differences
// Avoid using gvars here
static const double kDefaultDelta = 1e-8;

double EvaluateModelLogLik(ModelLikelihood& likelihood,
													 const proto::PayoffFeatureSet& featureset,
													 const Vec2& lambda) {
	DLOG << "Evaluating model likelihood at " << lambda;
	likelihood.Configure(lambda);
	BOOST_FOREACH(const proto::FrameWithFeatures& instance, featureset.frames()) {
		likelihood.Process(instance);
	}
	return likelihood.log_likelihood();
}

double EvaluateFeatureLogLik(FeatureLikelihood& likelihood,
														 const proto::PayoffFeatureSet& featureset,
														 const Vector<>& theta) {
	DLOG << "Evaluating feature likelihood at " << theta;
	likelihood.Configure(theta);
	BOOST_FOREACH(const proto::FrameWithFeatures& instance, featureset.frames()) {
		likelihood.Process(instance);
	}
	return likelihood.log_likelihood();
}

///////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
	// InitVars is part of base. We need because of the vars in dp_payoffs.cpp
	InitVars(argc, argv);

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
    ("help", "produce help message")
		("features", po::value<string>()->required(), "file containing payoff features")
		("weights", po::value<string>(), "feature weights")
    ("corner_penalty", po::value<float>()->required(), "per-corner penalty")
    ("occlusion_penalty", po::value<float>()->required(),
		 "additional penalty for occluding corners")
    ("with_gradient", "also compute gradient")
    ("logit", "use logistic likelihood (default is Gaussian)")
    ("delta", po::value<double>()->default_value(kDefaultDelta),
		 "Window size for finite differences.")
		;

	// Parse options
	po::variables_map opts;
	try {
		po::store(po::parse_command_line(argc, argv, desc), opts);
		po::notify(opts);
	} catch (const po::required_option& ex) {
		cout << "Missing required option: " << ex.get_option_name() << "\n" << desc << "\n";
		return 1;
	}
	if (opts.count("help")) {
    cout << desc << "\n";
    return 1;
	}

	// Load features
	fs::path features_file = opts["features"].as<string>();
	CHECK_PRED1(fs::exists, features_file);
	proto::PayoffFeatureSet feature_set;
	ReadLargeProto(features_file.string(), feature_set);

	// Construct parameters
	ManhattanHyperParameters params;
	params.corner_penalty = opts["corner_penalty"].as<float>();
	params.occlusion_penalty = opts["occlusion_penalty"].as<float>();
	params.weights = stream_to<VecF>(opts["weights"].as<string>());

	CHECK_GE(params.corner_penalty, 1e-18)
		<< "Penalties mut be positive (since constant in geometric series must be < 1)";
	CHECK_GE(params.occlusion_penalty, 1e-18)
		<< "Penalties mut be positive (since constant in geometric series must be < 1)";

	// TODO: finish implementing analytic gradients
	bool kEnableAnalyticalGradients = false;

	// Initialize likelihood functions
	Vec2 lambda = makeVector(params.corner_penalty, params.occlusion_penalty);
	Vector<> theta = asToon(params.weights);
	scoped_ptr<ModelLikelihood> model_lik(new ModelLikelihood(lambda));
	scoped_ptr<FeatureLikelihood> ftr_lik;
	if (opts.count("logit_likelihood")) {
		DLOG << "Using logistic feature likelihood";
		ftr_lik.reset(new LogitFeatureLikelihood(theta));
	} else {
		DLOG << "Using Gaussian feature likelihood";
		ftr_lik.reset(new GaussianFeatureLikelihood
									(theta, kEnableAnalyticalGradients));
	}

	// Prepare functions (for convenience with numerical gradientJacobians)
	boost::function<double(const Vec2&)> model_loglik_func =
		boost::bind(&EvaluateModelLogLik, ref(*model_lik), ref(feature_set), _1);
	boost::function<double(const Vector<>&)> ftr_loglik_func =
		boost::bind(&EvaluateFeatureLogLik, ref(*ftr_lik), ref(feature_set), _1);

	// Compute likelihood
	double model_loglik = model_loglik_func(lambda);
	double ftr_loglik = EvaluateFeatureLogLik(*ftr_lik, feature_set, theta);
	double loglik = ftr_loglik + model_loglik;

	// Done. Print with high precision so matlab can read with high precision.
	format fmt("%.18e ");
	DLOG << fmt % loglik;

	if (opts.count("with_gradient") > 0) {
		LogManager::Disable();
		double delta = opts["delta"].as<double>();
		Vec2 J_model_loglik = NumericalJacobian(model_loglik_func, lambda, delta);
		Vector<> J_ftr_loglik = NumericalJacobian(ftr_loglik_func, theta, delta);
		Vector<> numerical_J_loglik = concat(J_model_loglik, J_ftr_loglik);
		LogManager::Enable();

		/*Vector<> J_loglik = concat(model_lik->jacobian(), ftr_lik->jacobian());
		DLOG << format("%|5t|ANALYTIC %|30t|NUMERICAL %|55t|REL ERR\n");
		for (int i = 0; i < J_loglik.size(); i++) {
			double relerr = (J_loglik[i] - numerical_J_loglik[i]) / numerical_J_loglik[i];
			DLOG << format("%2d) %|5t|%f %|30t|%f %|55t|%f%%\n")
			  % i % J_loglik[i] % numerical_J_loglik[i] % (relerr*100);
			}*/

		for (int i = 0; i < numerical_J_loglik.size(); i++) {
			DLOG_N << fmt % numerical_J_loglik[i];
		}
		DLOG;  // end the line
	}

	return 0;
}
