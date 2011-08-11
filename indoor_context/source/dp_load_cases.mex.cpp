#include <unistd.h>

#include <vector>
#include <list>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

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
#include "landmark_payoffs.h"
#include "floorplan_renderer.h"
#include "building_features.h"

#include "matrix_traits.tpp"
#include "counted_foreach.tpp"

using namespace std;
using namespace indoor_context;
using boost::format;

mxArray* MatrixVecToMatlabArray(const vector<const MatF*> matrices) {
	CHECK(!matrices.empty());
	mxArray* arr = NewMatlabArray(matrices[0]->Rows(), matrices[0]->Cols(), matrices.size());
	double* p = mxGetPr(arr);
	for (int i = 0; i < matrices.size(); i++) {
		CHECK_SAME_SIZE(*matrices[i], *matrices[0]);
		// matlab arrays are stored column-major
		for (int x = 0; x < matrices[i]->Cols(); x++) {
			for (int y = 0; y < matrices[i]->Rows(); y++) {
				*p++ = (*matrices[i])[y][x];
			}
		}
	}
	return arr;
}


void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	if (nlhs > 2 || nrhs < 2 || nrhs > 3) {
		mexErrMsgTxt("Usage: cases=dp_load_cases(sequence_name, frame_ids[, feature_set])\n"); 
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
	double zceil = gt_map.floorplan().zceil();
	double zfloor = gt_map.floorplan().zfloor();

	// Create the struct array
	MatlabStructure cases = CaseProto.New(ids.size());
	plhs[0] = cases.get();

	// Feature generator (placed here to allow memory to be shared
	// across multiple invokations)
	//LineSweepFeatureGenerator gen;
	BuildingFeatures ftrgen(feature_set);

	// For rendering floorplans into the grid
	FloorPlanRenderer grid_renderer;

	// For holding features in grid coords
	ptr_vector<MatF> grid_features;

	// Generate features
	format case_name_fmt("%s:%d");
	COUNTED_FOREACH(int i, int id, ids) {
		string case_name = str(case_name_fmt % sequence_name % id);
		DLOG << "Loading frame " << id;
		mexEvalString("drawnow");

		// Load image
		KeyFrame& kf = *map.KeyFrameByIdOrDie(id);
		kf.LoadImage();
		kf.image.BuildMono();

		// Compute manhattan homology
		Mat3 fToC = GetFloorCeilHomology(kf.image.pc(), gt_map.floorplan());
		DPGeometry geom(kf.image.pc(), fToC);

		//
		// Compute ground truth
		//
		MatI gt_orients;
		int gt_num_walls, gt_num_occlusions;
		GetGroundTruth(gt_map.floorplan(), kf.image.pc(),
									 gt_orients, gt_num_walls, gt_num_occlusions);

		// Compute orientations in grid coordinates
		toon::Matrix<3,4> grid_cam = geom.imageToGrid * kf.image.pc().Linearize();
		grid_renderer.Render(gt_map.floorplan(), grid_cam, geom.grid_size);
		const MatI& gt_grid_orients = grid_renderer.GetOrientations();

		// Compute the mask for the path
		MatI gt_path(geom.ny(), geom.nx(), -1);
		for (int x = 0; x < geom.nx(); x++) {
			for (int y = 0; y < geom.ny(); y++) {
				if (gt_grid_orients[y][x] != kVerticalAxis) {
					gt_path[y == 0 ? y : y-1][x] = gt_grid_orients[y][x];
					break;
				}
			}
		}

		//
		// Generate pixel features
		//
		ftrgen.Compute(kf.image, &gt_orients);
		//mxArray* full_pixel_ftrs = MatrixVecToMatlabArray(ftrgen.features);  // TODO: remove

		// Convert to grid coords
		vector<const MatF*> grid_feature_ptrs;
		grid_features.resize(ftrgen.features.size());
		for (int j = 0; j < ftrgen.features.size(); j++) {
			geom.TransformDataToGrid(*ftrgen.features[j], grid_features[j]);
			grid_feature_ptrs.push_back(&grid_features[j]);
		}
		mxArray* pixel_ftrs = MatrixVecToMatlabArray(grid_feature_ptrs);

		//
		// Generate wall features
		//
		MatF agree, occl;
		ComputeLandmarkPayoffs(kf, geom, zfloor, zceil, agree, occl);
		vector<const MatF*> ms;
		ms.push_back(&agree);
		ms.push_back(&occl);
		mxArray* wall_ftrs = MatrixVecToMatlabArray(ms);

		//
		// Create geometry object
		//
		const PosedCamera& pc = kf.image.pc();
		MatlabStructure frame = FrameProto.New(1);
		frame.put(0, "camera_intrinsics", NewMatlabArrayFromMatrix(pc.camera().Linearize()));
		frame.put(0, "camera_extrinsics", NewMatlabArrayFromMatrix(pc.pose().ln().as_row()));
		frame.put(0, "floor_to_ceil", NewMatlabArrayFromMatrix(fToC));
		frame.put(0, "image", NewMatlabArrayFromImage(kf.image.rgb));

		//
		// Create ground truth
		//
		MatlabStructure gt = SolutionProto.New(1);
		gt.put(0, "orients", NewMatlabArrayFromMatrix(gt_grid_orients));
		gt.put(0, "path", NewMatlabArrayFromMatrix(gt_path));
		gt.put(0, "num_walls", NewMatlabArrayFromScalar(gt_num_walls));
		gt.put(0, "num_occlusions", NewMatlabArrayFromScalar(gt_num_occlusions));

		//
		// Add to output
		//
		cases.put(i, "sequence_name", NewMatlabArrayFromString(sequence_name));
		cases.put(i, "frame_id", NewMatlabArrayFromScalar(kf.id));
		cases.put(i, "image_file", NewMatlabArrayFromString(kf.image_file));
		cases.put(i, "ground_truth", gt.get());
		cases.put(i, "frame", frame.get());
		cases.put(i, "pixel_features", pixel_ftrs);
		//cases.put(i, "full_pixel_features", full_pixel_ftrs); // TODO: remove
		cases.put(i, "wall_features", wall_ftrs);
	}

	// Produce meta-information
	if (nlhs >= 2) {
		MatlabStructure meta = MetaProto.New(1);
		plhs[1] = meta.get();

		CHECK_EQ(ftrgen.features.size(), ftrgen.feature_strings.size());
		mwSize sz = ftrgen.feature_strings.size();
		mxArray* strs = mxCreateCellArray(1, &sz);
		TITLED("Features consist of:")
			for (int i = 0; i < ftrgen.feature_strings.size(); i++) {
				mxSetCell(strs, i, NewMatlabArrayFromString(ftrgen.feature_strings[i]));
				DLOG << i << ": " << ftrgen.feature_strings[i];
			}
		meta.put(0, "feature_strings", strs);
	}
}
