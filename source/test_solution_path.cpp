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

// Pass the ground truth orientations as the initial orientations
// Use this for testing the algorithm

// To be moved elsewhere?

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
		recon.OutputSolution(str(filepat % id % "soln"));

		// Compute accuracy
		double accuracy = recon.GetAccuracy(gt_map.floorplan());
		sum_accuracy += accuracy;
		num_frames++;
		DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

		// Get the path
		MatI path;
		recon.dp.ComputeSolutionPath(path);
		WriteMatrixImageRescaled(str(filepat % id % "path"), path);
		double score = 0.0;
		for (int y = 0; y < path.Rows(); y++) {
			const int* prow = path[y];
			for (int x = 0; x < path.Cols(); x++) {
				if (prow[x] >= 0) {
					score += recon.payoff_gen.payoffs.wall_scores[ prow[x] ][ y ][ x ];
				}
			}
		}
		DREPORT(score);
		DREPORT(recon.dp.solution.score);
		DREPORT(recon.dp.soln_num_walls);
		DREPORT(recon.dp.soln_num_occlusions);
		score -= recon.dp.soln_num_walls*recon.dp.payoffs->wall_penalty;
		score -= recon.dp.soln_num_occlusions*recon.dp.payoffs->occl_penalty;
		CHECK_EQ(score, recon.dp.solution.score);

		kf.UnloadImage();
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	return 0;
}
