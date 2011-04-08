#include <exception>
#include <iomanip>

#include <boost/exception/all.hpp>
#include <boost/filesystem.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/thread.hpp>

#include "entrypoint_types.h"
#include "joint_payoffs.h"
#include "manhattan_dp.h"
#include "map.h"
#include "bld_helpers.h"
#include "safe_stream.h"
#include "timer.h"
#include "payoff_helpers.h"
#include "canvas.h"
#include "numeric_utils.h"
#include "likelihoods.h"

#include "numerical_jacobian.tpp"
#include "format_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"

lazyvar<string> gvStereoOffsets("JointDP.Stereo.AuxOffsets");

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
	InitVars(argc, argv);
	Timer timer;

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
    ("help", "produce help message")
		("features", po::value<string>()->required(), "file containing payoff features")
		("weights", po::value<string>(), "feature weights")
    ("corner_penalty", po::value<float>()->required(), "per-corner penalty")
    ("occlusion_penalty", po::value<float>()->required(), "additional penalty for occluding corners")
    ("logit", "use logistic likelihood (default is Gaussian)")
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

	// Initialize map
	Map map;
	proto::TruthedMap gt_map;
	string cur_sequence = "";

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
		ftr_lik.reset(new GaussianFeatureLikelihood(theta));
	}

	static const double kDelta = 1e-6;

	// Compute feature likelihood and jacobian
	boost::function<double(const Vec2&)> model_loglik_func =
		boost::bind(&EvaluateModelLogLik, ref(*model_lik), ref(feature_set), _1);
	double model_loglik = model_loglik_func(lambda);
	Vec2 J_model_loglik = NumericalJacobian(model_loglik_func, lambda, kDelta);

	// Compute model likelihood and jacobian
	boost::function<double(const Vector<>&)> ftr_loglik_func =
		boost::bind(&EvaluateFeatureLogLik, ref(*ftr_lik), ref(feature_set), _1);
	double ftr_loglik = EvaluateFeatureLogLik(*ftr_lik, feature_set, theta);
	Vector<> J_ftr_loglik = NumericalJacobian(ftr_loglik_func, theta, kDelta);

	// Combine together
	double loglik = ftr_loglik + model_loglik;
	Vector<> J_loglik = concat(J_model_loglik, J_ftr_loglik);

	// Compute numerical jacobians
	//Vector<> num_J_model_loglik

	// Done. Print with high precision so matlab can read with high precision.
	DLOG << "Processed " << feature_set.frames_size() << " frames in " << timer;
	format fmt("%.18e ");
	DLOG << fmt % loglik;
	for (int i = 0; i < J_loglik.size(); i++) {
		DLOG_N << fmt % J_loglik[i];
	}
	DLOG;  // end the line

	return 0;
}
