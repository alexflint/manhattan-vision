// This file is a wrapper to evaluate parameter values given to it by
// a black box optimzier.

#include <boost/exception/all.hpp>
#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"
#include "joint_payoffs.h"
#include "manhattan_dp.h"
#include "map.h"
#include "bld_helpers.h"
#include "canvas.h"
#include "manhattan_ground_truth.h"
#include "safe_stream.h"
#include "timer.h"

#include "counted_foreach.tpp"
#include "format_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"

lazyvar<string> gvStereoOffsets("JointDP.Stereo.AuxOffsets");

int main(int argc, char **argv) {
	InitVars(argc, argv);
	AssertionManager::SetExceptionMode();

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
    ("help", "produce help message")
    ("mono_weight", po::value<float>()->required(), "weight for mono payoffs")
    ("stereo_weight", po::value<float>()->required(), "weight for stereo payoffs")
    ("3d_agreement_sigma", po::value<float>()->required(), "Gaussian bandiwth fro 3D agreement payoffs")
    ("3d_agreement_weight", po::value<float>()->required(), "weight for 3D agreement payoffs")
    ("3d_occlusion_weight", po::value<float>()->required(), "weight for 3D occlusion payoffs")
    ("corner_penalty", po::value<float>()->required(), "per-corner penalty")
    ("occlusion_penalty", po::value<float>()->required(), "additional penalty for occluding corners")
    ("sequence", po::value<vector<string> >()->required(), "sequences to evaluate")
    ("frame_stride", po::value<int>()->default_value(1),
		 "number of frames to skip between evaluations")
    ("loss", po::value<string>()->default_value("DepthError"),
		 "Loss function: DepthError or LabellingError")
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

	// Get params
	vector<string> sequences = opts["sequence"].as<vector<string> >();
	CHECK(!sequences.empty()) << "You must specify at least one sequence";

	int frame_stride = opts["frame_stride"].as<int>();
	CHECK_GT(frame_stride, 0);

	string loss_name = opts["loss"].as<string>();
	CHECK(loss_name == "DepthError" || loss_name == "LabellingError") << EXPR_STR(loss_name);
	bool use_depth_loss = (loss_name == "DepthError");

	// Initialize the payoff generator
	JointPayoffGen joint;
	vector<int> stereo_offsets = ParseMultiRange<int>(*gvStereoOffsets);
	vector<Vec3> point_cloud;  // must be outside scope as PointCloudPayoffs keeps a pointer

	// Load weights and penalties
	joint.point_cloud_gen.agreement_sigma = opts["3d_agreement_sigma"].as<float>();
	joint.agreement_weight = opts["3d_agreement_weight"].as<float>();
	joint.occlusion_weight = opts["3d_occlusion_weight"].as<float>();
	joint.stereo_weight = opts["stereo_weight"].as<float>();
	joint.mono_weight = opts["mono_weight"].as<float>();
	float corner_penalty = opts["corner_penalty"].as<float>();
	float occlusion_penalty = opts["occlusion_penalty"].as<float>();

	// Initialize the reconstructor
	ManhattanDPReconstructor recon;
	ManhattanGroundTruth gt;

	// Initialize statistics
	double sum_acc = 0;
	double sum_err = 0;
	int num_frames = 0;
	BOOST_FOREACH(const string& sequence, sequences) {
		TITLE("Loading " << sequence);
		// Load the map
		Map map;
		proto::TruthedMap gt_map;
		map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

		// Process each frame
		for (int i = 0; i < map.kfs.size(); i += frame_stride) {
			TITLE("Processing "<<sequence<<" frame "<<i);
			KeyFrame& frame = map.kfs[i];
			frame.LoadImage();

			// Setup geometry
			DPGeometryWithScale geom(frame.image.pc(),
															 gt_map.floorplan().zfloor(),
															 gt_map.floorplan().zceil());

			// Get point cloud
			point_cloud.clear();
			frame.GetMeasuredPoints(point_cloud);

			// Get auxiliary frames
			vector<const PosedImage*> aux_images;
			COUNTED_FOREACH(int i, int offset, stereo_offsets) {
				KeyFrame* aux_frame = map.KeyFrameById(frame.id+offset);
				if (aux_frame != NULL) {
					aux_frame->LoadImage();
					aux_images.push_back(&aux_frame->image);
				}
			}

			// Reconstruct
			joint.Compute(frame.image, geom, point_cloud, aux_images);
			joint.payoffs.wall_penalty = corner_penalty;
			joint.payoffs.occl_penalty = occlusion_penalty;
			recon.Compute(frame.image, geom, joint.payoffs);

			// Compute performance
			gt.Compute(gt_map.floorplan(), frame.image.pc());
			sum_acc += recon.ReportAccuracy(gt);
			sum_err += recon.ReportDepthError(gt);
			num_frames++;

			frame.UnloadImage();
		}
	}

	double av_depth_error = sum_err / num_frames;
	double av_labelling_error = 1. - sum_acc / num_frames;
	DLOG << fmt("AVERAGE DEPTH ERROR: %|40t|%.1f%%", av_depth_error*100.);
	DLOG << fmt("AVERAGE LABELLING ERROR: %|40t|%.1f%%", av_labelling_error*100.);

	if (use_depth_loss) {
		DLOG << av_depth_error;
	} else {
		DLOG << av_labelling_error;
	}
	
	return 0;
}
