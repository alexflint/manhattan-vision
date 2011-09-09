#include "entrypoint_types.h"
#include "likelihoods.h"
#include "likelihood_helpers.h"

#include "io_utils.tpp"

// Window size for finite differences
// Avoid using gvars here
static const double kDefaultDelta = 1e-8;

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
		return -1;
	}
	if (opts.count("help")) {
    cout << desc << "\n";
    return -1;
	}
	if (opts.count("with_gradient")) {
		cout << "GRADIENTS NOT IMPLEMENTED";
		return -1;
	}

	// Read command line arguments
	ManhattanHyperParameters params;
	params.corner_penalty = opts["corner_penalty"].as<float>();
	params.occlusion_penalty = opts["occlusion_penalty"].as<float>();
	params.weights = stream_to<VecF>(opts["weights"].as<string>());

	// Evaluate the likelihood
	double loglik = EvaluateLikelihood(params,
																		 opts["features"].as<string>(),
																		 opts.count("logit_likelihood")>0);

	// Done. Print with high precision so matlab can read with high precision.	
	DLOG << format("%.18e ") % loglik;

	return 0;
}
