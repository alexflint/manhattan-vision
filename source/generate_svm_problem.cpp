#include <unistd.h>

#include <vector>
#include <list>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <VW/Image/imagecopy.h>

#include "entrypoint_types.h"
#include "common_types.h"
#include "manhattan_dp.h"
#include "map.h"
#include "map.pb.h"
#include "bld_helpers.h"
#include "line_sweep_features.h"
#include "landmark_payoffs.h"
#include "floorplan_renderer.h"
#include "building_features.h"

#include "io_utils.tpp"
#include "counted_foreach.tpp"

using namespace std;
using namespace indoor_context;
using boost::format;

static const int kStride = 3;

int main(int argc, char **argv) {
	InitVars();

	if (argc < 3 || argc > 4) {
		cout << "Usage: generate_svm_problem SEQUENCE FRAME_IDS [FEATURE_SET]" << endl;
		return 0;
	}

	// Load the data
	format case_tpl("%s:%d");
	fs::path sequences_dir("sequences");
	fs::path rel_map_path("ground_truth/truthed_map.pro");

	string sequence_name = argv[1];
	fs::path map_file = sequences_dir/sequence_name/rel_map_path;

	vector<int> ids = ParseMultiRange<int>(argv[2]);

	string feature_set = argc >= 4 ? argv[3] : "default";

	// Load the sequence
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(map_file.string(), gt_map);
	double zceil = gt_map.floorplan().zceil();
	double zfloor = gt_map.floorplan().zfloor();

	// Feature generator (placed here to allow memory to be shared
	// across multiple invokations)
	BuildingFeatures ftrgen(feature_set);

	// For rendering floorplans into the grid
	//FloorPlanRenderer grid_renderer;

	// For holding features in grid coords
	ptr_vector<MatF> grid_features;

	// Open file streams
	ofstream problem_out[3];
	format file_fmt("classifier/%s/%s_class%d_vs_all.problem");
	for (int i = 0; i < 3; i++) {
		string frame = ids.size() == 1 ? "frame"+itoa(ids[0],3) : "multiframe";
		string prob_file = str(file_fmt % sequence_name % frame % i);
		DREPORT(prob_file);
		problem_out[i].open(prob_file.c_str());
	}

	// Generate features
	format case_name_fmt("%s:%d:%d:%d");
	BOOST_FOREACH(int id, ids) {
		DLOG << "Loading frame " << id;

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
		/*toon::Matrix<3,4> grid_cam = geom.imageToGrid * kf.image.pc().Linearize();
		grid_renderer.Render(gt_map.floorplan(), grid_cam, geom.grid_size);
		const MatI& gt_grid_orients = grid_renderer.GetOrientations();*/

		// Compute the mask for the path
		/*MatI gt_path(geom.ny(), geom.nx(), -1);
		for (int x = 0; x < geom.nx(); x++) {
			for (int y = 0; y < geom.ny(); y++) {
				if (gt_grid_orients[y][x] != kVerticalAxis) {
					gt_path[y == 0 ? y : y-1][x] = gt_grid_orients[y][x];
					break;
				}
			}
			}*/

		//
		// Generate pixel features
		//
		ftrgen.Compute(kf.image, &gt_orients);
		for (int y = 0; y < gt_orients.Rows(); y += kStride) {
			const int* gtrow = gt_orients[y];
			for (int x = 0; x < gt_orients.Cols(); x += kStride) {
				for (int i = 0; i < 3; i++) {
					problem_out[i] << (gtrow[x] == i ? "+1" : "-1");
					COUNTED_FOREACH(int j, const MatF* ftr, ftrgen.features) {
						problem_out[i] << " " << (j+1) << ":" << (*ftr)[y][x];
					}
					problem_out[i] << "\n";
				}
			}
		}
					

		// Convert to grid coords
		/*vector<const MatF*> grid_feature_ptrs;
		grid_features.resize(ftrgen.features.size());
		for (int j = 0; j < ftrgen.features.size(); j++) {
			geom.TransformDataToGrid(*ftrgen.features[j], grid_features[j]);
			grid_feature_ptrs.push_back(&grid_features[j]);
		}
		mxArray* pixel_ftrs = MatrixVecToMatlabArray(grid_feature_ptrs);*/

		//
		// Generate wall features
		//
		/*MatF agree, occl;
		ComputeLandmarkPayoffs(kf, geom, zfloor, zceil, agree, occl);
		vector<const MatF*> ms;
		ms.push_back(&agree);
		ms.push_back(&occl);
		mxArray* wall_ftrs = MatrixVecToMatlabArray(ms);*/

		//
		// Create geometry object
		//
		/*const PosedCamera& pc = kf.image.pc();
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
		cases.put(i, "wall_features", wall_ftrs);*/
	}

	return 0;
}
