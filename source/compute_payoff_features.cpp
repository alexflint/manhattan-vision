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

#include "format_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"

lazyvar<string> gvStereoOffsets("JointDP.Stereo.AuxOffsets");

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Timer timer;

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
    ("help", "produce help message")
    ("sequence", po::value<vector<string> >()->required(), "name of training sequence")
		("frame_stride", po::value<int>()->default_value(1), "number of frames to skip between evaluations")
		("output", po::value<string>()->required(), "output file for features")
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

	// Deal with params
	vector<string> sequences = opts["sequence"].as<vector<string> >();
	CHECK(!sequences.empty()) << "You must specify at least one sequence with --sequence=<name>";

	int frame_stride = opts["frame_stride"].as<int>();
	CHECK_GT(frame_stride, 0);

	fs::path features_file = opts["output"].as<string>();
	CHECK_PRED1(fs::exists, features_file.parent_path());

	vector<int> stereo_offsets = ParseMultiRange<int>(*gvStereoOffsets);

	// Initialize the protocol buffer
	proto::PayoffFeatureSet featureset;

	// Process each sequence
	int num_frames = 0;
	JointPayoffGen joint;
	vector<Vec3> point_cloud;
	Map maps[sequences.size()];
	proto::TruthedMap gt_maps[sequences.size()];
	for (int i = 0; i < sequences.size(); i++) {
		TITLE("Processing " << sequences[i]);
		maps[i].LoadWithGroundTruth(GetMapPath(sequences[i]), gt_maps[i]);

		for (int j = 0; j < maps[i].kfs.size(); j += frame_stride) {
			KeyFrame& frame = maps[i].kfs[j];
			frame.LoadImage();
			num_frames++;

			TITLE("Processing frame " << frame.id);

			// Setup geometry
			DPGeometryWithScale geometry(frame.image.pc(),
																	 gt_maps[i].floorplan().zfloor(),
																	 gt_maps[i].floorplan().zceil());

			// Load point cloud
			point_cloud.clear();
			frame.GetMeasuredPoints(point_cloud);

			// Load auxiliary frames
			vector<const PosedImage*> aux_images;
			vector<int> aux_ids;
			BOOST_FOREACH(int offset, stereo_offsets) {
				Frame* aux_frame = frame.map->KeyFrameById(frame.id+offset);
				if (aux_frame != NULL) {
					aux_frame->LoadImage();
					aux_images.push_back(&aux_frame->image);
					aux_ids.push_back(frame.id+offset);
				}
			}

			// Compute payoffs
			joint.Compute(frame.image, geometry, point_cloud, aux_images);

			// Add an entry to the protocol buffer
			proto::FrameWithFeatures& framedata = *featureset.add_frames();
			framedata.set_sequence(sequences[i]);
			framedata.set_frame_id(frame.id);
			PackFeatures(aux_ids, joint, *framedata.mutable_features());
		}
	}

	// Store payoffs
	TIMED("Serializing") WriteProto(features_file.string(), featureset);

	DLOG << "Processed " << num_frames << " in " << timer;

	return 0;
}
