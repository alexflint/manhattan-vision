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
		("store_features", "if 1, store all features in the output file")
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

	bool store_features = opts.count("store_features");

	vector<int> stereo_offsets = ParseMultiRange<int>(*gvStereoOffsets);

	// Initialize the protocol buffer
	proto::PayoffFeatureSet featureset;
	ManhattanGroundTruth gt;

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
			int nx = geometry.grid_size[0];
			int ny = geometry.grid_size[1];

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
			DPPayoffs sum_stereo(geometry.grid_size);
			BOOST_FOREACH(const StereoPayoffGen& gen, joint.stereo_gens) {
				sum_stereo.Add(gen.payoffs, 1. / joint.stereo_gens.size());
			}

			// Get ground truth path
			VecI path(nx, 0);
			VecI orients(nx, 0);
			gt.Compute(gt_maps[i].floorplan(), frame.image.pc());
			gt.ComputePath(geometry, path, orients);

			// Trace features along path
			MatF path_ftrs(4, nx);
			for (int x = 0; x < nx; x++) {
				path_ftrs[0][x] = joint.mono_gen.payoffs.wall_scores[ orients[x] ][ path[x] ][ x ];
				path_ftrs[1][x] = sum_stereo.wall_scores[ orients[x] ][ path[x] ][ x ];
				path_ftrs[2][x] = joint.point_cloud_gen.agreement_payoffs[ path[x] ][ x ];
				path_ftrs[3][x] = joint.point_cloud_gen.occlusion_payoffs[ path[x] ][ x ];
			}
			
			// Add an entry to the protocol buffer
			proto::FrameWithFeatures& framedata = *featureset.add_frames();
			framedata.set_sequence(sequences[i]);
			framedata.set_frame_id(frame.id);
			framedata.set_num_walls(gt.num_walls());
			framedata.set_num_occlusions(gt.num_occlusions());

			Matrix<-1,-1,int> mpath = asToon(path).as_col();
			PackMatrix(asVNL(mpath), *framedata.mutable_path());
			Matrix<-1,-1,int> morients = asToon(orients).as_col();
			PackMatrix(asVNL(morients), *framedata.mutable_orients());

			PackMatrix(path_ftrs, *framedata.mutable_path_features());

			if (store_features) {
				PackFeatures(joint, framedata);
			}
		}
	}

	// Store payoffs
	TIMED("Serializing") WriteProto(features_file.string(), featureset);

	DLOG << "Processed " << num_frames << " frames in " << timer;

	return 0;
}
