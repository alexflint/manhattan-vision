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
#include "canvas.h"

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
		recon.OutputSolution(str(filepat % id % "soln"));

		// Compute accuracy
		double accuracy = recon.GetAccuracy(gt_map.floorplan());
		sum_accuracy += accuracy;
		num_frames++;
		DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

		// Get the path
		MatI path;
		recon.dp.ComputeSolutionPath(path);
		//WriteMatrixImageRescaled(str(filepat % id % "path"), path);
		double score = 0.0;
		for (int y = 0; y < path.Rows(); y++) {
			const int* prow = path[y];
			for (int x = 0; x < path.Cols(); x++) {
				if (prow[x] >= 0) {
					score += recon.payoff_gen.payoffs.wall_scores[ prow[x] ][ y ][ x ];
				}
			}
		}
		score -= recon.dp.soln_num_walls*recon.dp.payoffs->wall_penalty;
		score -= recon.dp.soln_num_occlusions*recon.dp.payoffs->occl_penalty;
		CHECK_EQ(score, recon.dp.solution.score);

		WriteImage(str(filepat % id % "orig"), kf.image.rgb);


		MatF po = recon.payoff_gen.payoffs.wall_scores[0] + recon.payoff_gen.payoffs.wall_scores[1];
		FileCanvas po_canvas(str(filepat % id % "payoffs"), recon.geometry.grid_size);
		po_canvas.DrawImageRescaled(po);

		// Draw the path
		FileCanvas path_canvas(str(filepat % id % "path"), recon.geometry.grid_size);
		path_canvas.DrawImageRescaled(po);//recon.payoff_gen.payoffs.wall_scores[0]);
		path_canvas.SetLineWidth(6.0);
		path_canvas.SetColor(Colors::blue());
		const DPState& first = *recon.dp.full_backtrack[0];
		path_canvas.MoveTo(makeVector(1.0*first.col, first.row));
		for (int i = 1; i < recon.dp.full_backtrack.size(); i++) {
			const DPState& cur = *recon.dp.full_backtrack[i];
			path_canvas.LineTo(makeVector(1.0*cur.col, cur.row));
		}
		path_canvas.Stroke();

		// Draw wireframe model
		FileCanvas wf_canvas(str(filepat % id % "wireframe"), recon.geometry.grid_size);
		wf_canvas.DrawImageRescaled(po);//recon.payoff_gen.payoffs.wall_scores[0]);
		wf_canvas.SetLineWidth(6.0);
		wf_canvas.SetColor(Colors::aqua());
		BOOST_FOREACH(const LineSeg& seg, recon.dp.soln_segments) {
			wf_canvas.MoveTo(seg.start);
			wf_canvas.LineTo(seg.end);
			wf_canvas.LineTo(recon.geometry.Transfer(seg.end));
			wf_canvas.LineTo(recon.geometry.Transfer(seg.start));
			wf_canvas.LineTo(seg.start);
			wf_canvas.Stroke();
		}

		wf_canvas.SetColor(Colors::blue());
		wf_canvas.MoveTo(makeVector(1.0*first.col, first.row));
		for (int i = 1; i < recon.dp.full_backtrack.size(); i++) {
			const DPState& cur = *recon.dp.full_backtrack[i];
			wf_canvas.LineTo(makeVector(1.0*cur.col, cur.row));
		}
		wf_canvas.Stroke();

		kf.UnloadImage();
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	return 0;
}
