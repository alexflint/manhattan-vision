#include <unistd.h>

#include <vector>
#include <list>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <google/heap-profiler.h>
#include <google/profiler.h>

#include <mex.h>

#include <VW/Image/imagecopy.h>

#include "common_types.h"
#include "manhattan_dp.h"
#include "map.h"
#include "map.pb.h"
#include "matlab_utils.h"
#include "bld_helpers.h"
#include "dp_structures.h"
#include "line_sweep_features.h"

#include "counted_foreach.tpp"

using namespace std;
using namespace indoor_context;
using boost::format;

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	//ProfilerStart("/tmp/dp_load_cases.prof");

	InitMex();
	if (nlhs != 1 || nrhs < 2 || nrhs > 3) {
		mexErrMsgTxt("Usage: cases=dp_load_cases(sequence_name, ids_to_load, [feature_set])"); 
	}

	// Load the data
	format case_tpl("%s:%d");
	//vector<string> cases;
	fs::path sequences_dir("sequences");
	fs::path rel_map_path("ground_truth/truthed_map.pro");

	string sequence_name = MatlabArrayToString(prhs[0]);
	fs::path map_file = sequences_dir/sequence_name/rel_map_path;

	VecD ids = MatlabArrayToVector(prhs[1]);

	string feature_set = nrhs >= 3 ? MatlabArrayToString(prhs[2]) : "default";

	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(map_file.string(), gt_map);

	// Create the struct array
	MatlabStructure cases = CaseProto.New(ids.size());
	plhs[0] = cases.get();

	// Feature generator (placed here to allow memory to be shared
	// across multiple invokations)
	LineSweepFeatureGenerator gen;

	// Generate features
	format case_name_fmt("%s:%d");
	COUNTED_FOREACH(int i, int id, ids) {
		string case_name = str(case_name_fmt % sequence_name % id);
		DLOG << "Loading frame " << id;

		// Copy to matrix form
		KeyFrame& kf = *map.KeyFrameByIdOrDie(id);
		kf.LoadImage();
		kf.image.BuildMono();

		// Compute manhattan homology
		Mat3 fToC = GetFloorCeilHomology(kf.image.pc(), gt_map.floorplan());

		// Compute ground truth
		MatI gt_orients;
		int gt_num_walls, gt_num_occlusions;
		GetGroundTruth(gt_map.floorplan(), kf.image.pc(),
									 gt_orients, gt_num_walls, gt_num_occlusions);

		// Generate features
		mxArray* ftrs = NULL;
		if (feature_set == "default" || feature_set == "line_sweep") {
			gen.Compute(kf.image);
			int ftr_size = LineSweepFeatureGenerator::kFeatureLength;
			ftrs = NewMatlabArray(kf.image.ny(), kf.image.nx(), ftr_size);
			double* ftr_data = mxGetPr(ftrs);
			for (int j = 0; j < ftr_size; j++) {
				for (int x = 0; x < kf.image.nx(); x++) {
					for (int y = 0; y < kf.image.ny(); y++) {
						const LineSweepFeatureGenerator::FeatureVec* row = &gen.features[y*kf.image.nx()];
						*ftr_data++ = gen.features[y*kf.image.nx()+x][j];
					}
				}
			}

		} else if (feature_set == "gt") {
			int ftr_size = 3;
			ftrs = NewMatlabArray(kf.image.ny(), kf.image.nx(), ftr_size);
			double* ftr_data = mxGetPr(ftrs);
			for (int j = 0; j < 3; j++) {
				for (int x = 0; x < kf.image.nx(); x++) {
					for (int y = 0; y < kf.image.ny(); y++) {
						*ftr_data++ = gt_orients[y][x] == j ? 1.0 : 0.0;
					}
				}
			}

		} else if (feature_set == "sweeps_only") {
			int ftr_size = 3;
			ftrs = NewMatlabArray(kf.image.ny(), kf.image.nx(), ftr_size);
			IsctGeomLabeller line_sweeper(kf.image);
			double* ftr_data = mxGetPr(ftrs);
			for (int j = 0; j < 3; j++) {
				for (int x = 0; x < kf.image.nx(); x++) {
					for (int y = 0; y < kf.image.ny(); y++) {
						*ftr_data++ = line_sweeper.orient_map[y][x] == j ? 1.0 : 0.0;
					}
				}
			}
		}

		CHECK_NOT_NULL(ftrs) << "Unrecognised feature set: '" << feature_set << "'";

		// Create geometry object
		const PosedCamera& pc = kf.image.pc();
		MatlabStructure frame = FrameProto.New(1);
		frame.put(0, "camera_intrinsics", NewMatlabArrayFromMatrix(pc.camera().Linearize()));
		frame.put(0, "camera_extrinsics", NewMatlabArrayFromMatrix(pc.pose().ln().as_row()));
		frame.put(0, "floor_to_ceil", NewMatlabArrayFromMatrix(fToC));
		frame.put(0, "image", NewMatlabArrayFromImage(kf.image.rgb));

		// Create ground truth
		MatlabStructure gt = SolutionProto.New(1);
		gt.put(0, "orients", NewMatlabArrayFromMatrix(gt_orients));
		gt.put(0, "num_walls", NewMatlabArrayFromScalar(gt_num_walls));
		gt.put(0, "num_occlusions", NewMatlabArrayFromScalar(gt_num_occlusions));

		// Set fields
		cases.put(i, "sequence_name", NewMatlabArrayFromString(sequence_name));
		cases.put(i, "frame_id", NewMatlabArrayFromScalar(kf.id));
		cases.put(i, "image_file", NewMatlabArrayFromString(kf.image_file));
		cases.put(i, "ground_truth", gt.get());
		cases.put(i, "frame", frame.get());
		cases.put(i, "features", ftrs);
	}

	//ProfilerStop();
}
