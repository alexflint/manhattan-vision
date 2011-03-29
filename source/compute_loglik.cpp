#include <exception>

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

#include "format_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"

static const int kDimW = 4;
static const int kDimLambda = 2;
static const int kDimTheta = kDimW + kDimLambda;

lazyvar<string> gvStereoOffsets("JointDP.Stereo.AuxOffsets");

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Timer timer;

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
    ("help", "produce help message")
		("features", po::value<string>()->required(), "file containing payoff features")

    ("mono_weight", po::value<float>()->required(), "weight for mono payoffs")
    ("stereo_weight", po::value<float>()->required(), "weight for stereo payoffs")
    ("3d_agreement_weight", po::value<float>()->required(), "weight for 3D agreement payoffs")
    ("3d_occlusion_weight", po::value<float>()->required(), "weight for 3D occlusion payoffs")
    ("corner_penalty", po::value<float>()->required(), "per-corner penalty")
    ("occlusion_penalty", po::value<float>()->required(), "additional penalty for occluding corners")
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
	proto::PayoffFeatureSet featureset;
	ReadLargeProto(features_file.string(), featureset);

	// Construct parameters
	JointPayoffParameters params;
	params.corner_penalty = opts["corner_penalty"].as<float>();
	params.occlusion_penalty = opts["occlusion_penalty"].as<float>();
	params.mono_weight = opts["mono_weight"].as<float>();
	params.stereo_weight = opts["stereo_weight"].as<float>();
	params.agreement_weight = opts["3d_agreement_weight"].as<float>();
	params.occlusion_weight = opts["3d_occlusion_weight"].as<float>();

	CHECK_GE(params.corner_penalty, 1e-17)
		<< "Constant in geometric series must be < 1, so penalty must be > 0";
	CHECK_GE(params.occlusion_penalty, 1e-17)
		<< "Constant in geometric series must be < 1, so penalty must be > 0";

	// The order of this vector must match that in compute_loglik.m and CompilePayoffs()
	Vector<> wts = makeVector(params.mono_weight,
														params.stereo_weight,
														params.agreement_weight,
														params.occlusion_weight);
	double wts_norm = norm(wts);
	double wts_norm_sq = norm_sq(wts);

	// Initialize map
	Map map;
	proto::TruthedMap gt_map;
	string cur_sequence = "";
	int num_frames = 0;

	// Initialize function value and jacobian
	double loglik = 0;
	Vector<> J_loglik(wts.size() + 2);
	J_loglik = Zeros;
	DPPayoffs payoffs;
	ptr_vector<DPPayoffs> J_payoffs;

	// Process each frame
	BOOST_FOREACH(const proto::FrameWithFeatures& framedata, featureset.frames()) {
		TITLE("Processing " << framedata.sequence() << ":" << framedata.frame_id());
		num_frames++;

		// Load frame
		if (framedata.sequence() != cur_sequence) {
			map.LoadWithGroundTruth(GetMapPath(framedata.sequence()), gt_map);
			cur_sequence = framedata.sequence();
		}
		KeyFrame& frame = *map.KeyFrameById(framedata.frame_id());
		frame.LoadImage();

		// Get ground truth
		ManhattanGroundTruth gt(gt_map.floorplan(), frame.image.pc());
		DPGeometryWithScale geom(frame.image.pc(),
														 gt_map.floorplan().zfloor(),
														 gt_map.floorplan().zceil());

		// Compile payoffs
		PayoffFeatures features;
		UnpackFeatures(framedata.features(), features);
		CompilePayoffs(features, params, payoffs, &J_payoffs);
		CHECK_EQ(geom.grid_size, matrix_size(payoffs)) << "Loaded payoffs were not of size ManhattanDP.GridSize";

		const vector<LineSeg>& segs =
			(geom.horizon_row < .5*geom.grid_size[1]) ?
			gt.floor_segments :
			gt.ceil_segments;

		// Compute the solution path
		VecI path(payoffs.nx(), 0);
		VecI orients(payoffs.nx(), 0);
		for (int i = 0; i < segs.size(); i++) {
			Vec3 grid_eqn = geom.gridToImage.T() * segs[i].eqn();  // for lines we apply H^-T (inverse of transpose)
			float x0 = geom.ImageToGrid(segs[i].start)[0];
			float x1 = geom.ImageToGrid(segs[i].end)[0];
			if (x0 > x1) {
				swap(x0, x1);
			}
			for (int x = floori(x0); x <= ceili(x1); x++) {
				Vec3 isct = grid_eqn ^ makeVector(1, 0, -x);
				int y = roundi(project(isct)[1]);
				// This get messy near the edge of line segments that don't meet perfectly
				path[x] = Clamp<int>(y, 0, payoffs.ny()-1);
				orients[x] = gt.segment_orients[i];
			}
		}
		for (int x = 0; x < payoffs.nx(); x++) {
			CHECK(path[x] != -1) << "Path does not fully span image " << EXPR_STR(x);
			CHECK_INTERVAL(orients[x], 0, 1) << "Invalid orientation " << EXPR_STR(x);
		}

		// Sum the features over the path. Normalization is important
		// here. It can be shown (though not trivially) that for logistic
		// P(f|m,p), dividing by the norm of the parameter vector means
		// that integrating P(f|m,p) over f results some constant that is
		// independent of p). This means that the log-likelihood for each
		// column needs to have log(norm(p)) subtracted.
		double sum_logit = 0;
		Vec4 J_sum_logit = Zeros;
		for (int x = 0; x < payoffs.nx(); x++) {
			double y = payoffs.wall_scores[ orients[x] ][ path[x] ][x];
			sum_logit += log(1. + exp(-y));
			for (int k = 0; k < 4; k++) {
				double J_y = J_payoffs[k].wall_scores[ orients[x] ][ path[x] ][x];
				J_sum_logit[k] += J_y * exp(-y) / (1. + exp(-y));
			}
		}
		// This is important when doing inference on the parameters as it
		// renders the integral over the feature space independent of the
		// choice of parameters...
		double ftr_loglik = -payoffs.nx()*log(wts_norm) - sum_logit;
		Vec4 J_ftr_loglik = -payoffs.nx()*wts/wts_norm_sq + J_sum_logit;

		// Compute model log-likelihood: 
		double n1 = gt.num_walls();
		double n2 = gt.num_occlusions();
		double lambda1 = params.corner_penalty;
		double lambda2 = params.occlusion_penalty;
		double log_top = -lambda1*n1 + -lambda2*n2;
		double exp1 = exp(-lambda1);
		double exp2 = exp(-lambda2);
		double exp12 = exp(-lambda1-lambda2);
		double bottom = 1. - exp1 - exp2 + exp12;
		CHECK_GT(bottom, 1e-17) << "Was a penalty less than zero?";
		double model_loglik = log_top - log(bottom);

		// Compute derivative of model log-likelihood
		Vec2 J_model_loglik;
		J_model_loglik[0] = -n1 - (exp1-exp12)/bottom;
		J_model_loglik[1] = -n2 - (exp2-exp12)/bottom;

		// temp
		DLOG << model_loglik;
		DLOG << J_model_loglik;
		DLOG << ftr_loglik;
		DLOG << J_ftr_loglik;
		return 0;

		// Accumulate
		loglik += model_loglik + ftr_loglik;
		J_loglik.slice<0,2>() += J_model_loglik;
		J_loglik.slice<2,4>() += J_ftr_loglik;
	}

	// Compute element-wise log of Jacobian of the likelihood (not the
	// same as the Jacobian of the log-likelihood!)

	// Done
	DLOG << "Processed " << num_frames << " frames in " << timer;
	DLOG << loglik;
	DLOG << J_loglik;

	return 0;
}
