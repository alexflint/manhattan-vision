#include "entrypoint_types.h"
#include "manhattan_dp.h"
#include "vars.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"
#include "timer.h"
#include "clipping.h"
#include "bld_helpers.h"
#include "safe_stream.h"
#include "line_sweep_features.h"

#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG << "Usage: " << argv[0] << " SEQUENCE FRAMES";
		return -1;
	}

	const char* sequence = argv[1];
	const vector<int> ids = ParseMultiRange<int>(argv[2]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	// Set up the reconstruction
	ManhattanDPReconstructor recon;

	float sum_accuracy = 0.0;
	int num_frames = 0;
	BOOST_FOREACH(int id, ids) {
		TITLE("Processing frame " << id);
		KeyFrame& kf = *map.KeyFrameByIdOrDie(id);
		kf.LoadImage();

		// Compute the cost function
		LineSweepDPScore gen(kf.image);

		// Perform the reconstruction
		Mat3 fToC = GetFloorCeilHomology(kf.image.pc(), gt_map.floorplan());
		recon.Compute(kf.image, fToC, gen.objective);

		// Draw some vizualizations
		format filepat("out/frame%02d_%s.png");
		//recon.OutputSolution(str(filepat % id % "soln"));

		// Compute accuracy
		double accuracy = recon.GetAccuracy(gt_map.floorplan());
		sum_accuracy += accuracy;
		num_frames++;
		DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

		// Get the exact orientations
		MatI exact_orients;
		TIMED("Compute exact orientations") recon.dp.ComputeExactOrients(exact_orients);

		// Get the path
		MatI old_path, path;
		recon.dp.ComputeSolutionPath(path);
		recon.dp.ComputeSolutionPathOld(old_path);

		CHECK_SAME_SIZE(path, old_path);
		for (int y = 0; y < path.Rows(); y++) {
			for (int x = 0; x < path.Cols(); x++) {
				if (path[y][x] != old_path[y][x]) {
					DLOG << "path differs from old_path:\n"
							 << "  path["<<y<<"]["<<x<<"] = " << path[y][x] << "\n"
							 << "  old_path["<<y<<"]["<<x<<"] = " << old_path[y][x];
				}
			}
		}

		// Sum score over pixels
		double pixel_score = 0.0;
		for (int y = 0; y < exact_orients.Rows(); y++) {
			const int* orientrow = exact_orients[y];
			for (int x = 0; x < exact_orients.Cols(); x++) {
				pixel_score += gen.objective.pixel_scores[ orientrow[x] ][y][x];
			}
		}
		pixel_score -= recon.dp.soln_num_walls*recon.dp.payoffs->wall_penalty;
		pixel_score -= recon.dp.soln_num_occlusions*recon.dp.payoffs->occl_penalty;
		CHECK_EQ(pixel_score, recon.dp.solution.score);

		// Sum score along path
		recon.dp.ComputeSolutionPath(path);
		double path_score = 0.0;
		for (int y = 0; y < path.Rows(); y++) {
			const int* prow = path[y];
			for (int x = 0; x < path.Cols(); x++) {
				if (prow[x] >= 0) {
					path_score += recon.payoff_gen.payoffs.wall_scores[ prow[x] ][ y ][ x ];
				}
			}
		}
		path_score -= recon.dp.soln_num_walls*recon.dp.payoffs->wall_penalty;
		path_score -= recon.dp.soln_num_occlusions*recon.dp.payoffs->occl_penalty;
		CHECK_EQ(path_score, recon.dp.solution.score);

		DREPORT(pixel_score, path_score, recon.dp.solution.score);

		MatI grid_orients;
		recon.dp.ComputeGridOrients(grid_orients);
		WriteOrientationImage(str(filepat % id % "grid_orients"), grid_orients);

		// Output exact versus approx orients
		WriteOrientationImage(str(filepat % id % "exact_orients"), exact_orients);
		WriteOrientationImage(str(filepat % id % "approx_orients"), recon.dp.soln_orients);

		MatI diff(exact_orients.Rows(), exact_orients.Cols());
		for (int y = 0; y < diff.Rows(); y++) {
			for (int x = 0; x < diff.Cols(); x++) {
				diff[y][x] = exact_orients[y][x] == recon.dp.soln_orients[y][x] ? 0 : 1;
			}
		}
		WriteMatrixImageRescaled(str(filepat % id % "diff"), diff);

		kf.UnloadImage();
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	return 0;
}
