#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include "entrypoint_types.h"
#include "numeric_utils.h"
#include "likelihoods.h"
#include "map.h"
#include "map.pb.h"
#include "map_io.h"
#include "bld_helpers.h"
#include "manhattan_dp.h"

#include "protobuf_utils.tpp"
#include "vector_utils.tpp"

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

	CHECK_GE(params.corner_penalty, 1e-18)
		<< "Penalties mut be positive (since constant in geometric series must be < 1)";
	CHECK_GE(params.occlusion_penalty, 1e-18)
		<< "Penalties mut be positive (since constant in geometric series must be < 1)";

	// Load Features
	string features_file = opts["features"].as<string>();
	CHECK_PRED1(fs::exists, features_file);
	proto::PayoffFeatureSet feature_set;
	ReadLargeProto(features_file, feature_set);

	// Initialize likelihood functions
	Vec2 lambda = makeVector(params.corner_penalty, params.occlusion_penalty);
	Vector<> theta = asToon(params.weights);
	ModelLikelihood model_likelihood(lambda);
	GaussianFeatureLikelihood ftr_likelihood(theta, false);

	// Load map for first instance

	// TODO: store frames and geometry (actually just zfloor and zceil)
	// in the protocol buffer to avoid needing to load maps at test time
	Map map;
	proto::TruthedMap gt_map;
	LoadXmlMapWithGroundTruth(GetMapPath(feature_set.frames(0).sequence()),
														map,
														gt_map);

	// Compute the prior term
	DPPayoffs payoffs;
	model_likelihood.PopulatePayoffs(payoffs);

	// Process each frame
	double sum_acc = 0;
	double sum_err = 0;
	BOOST_FOREACH(const proto::FrameWithFeatures& instance, feature_set.frames()) {
		CHECK(instance.sequence() == feature_set.frames(0).sequence())
			<< "Multiple sequences not supported yet";

		Frame* frame = map.GetFrameByIdOrDie(instance.frame_id());
		DPGeometryWithScale geom(frame->image.pc(),
														 gt_map.floorplan().zfloor(),
														 gt_map.floorplan().zceil());

		ftr_likelihood.ComputePayoffs(instance, payoffs);

		ManhattanDPReconstructor recon;
		recon.Compute(frame->image, geom, payoffs);
		const DPSolution& soln = recon.dp.solution;

		ManhattanGroundTruth gt(gt_map.floorplan(), frame->image.pc());
		double pixel_acc = recon.ReportAccuracy(gt) * 100;
		sum_acc += pixel_acc;
		double mean_err = recon.ReportDepthError(gt) * 100;
		sum_err += mean_err;

			// Draw the solution
		recon.OutputSolution(str(format("out/solution_%02d.png") % instance.frame_id()));
	}

  // Note: these are already multipled by 100!
	int num_frames = feature_set.frames_size();
	double av_err = sum_err / num_frames;
	double av_labelling_err = 100. - sum_acc / num_frames;
	DLOG << format("AVERAGE DEPTH ERROR: %|40t|%.1f%%") % av_err;
	DLOG << format("AVERAGE LABELLING ERROR: %|40t|%.1f%%") % av_labelling_err;

	return 0;
}
