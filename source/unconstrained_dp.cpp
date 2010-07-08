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

using namespace indoor_context;
using namespace toon;

// To be moved elsewhere?
lazyvar<int> gvFrameId("ManhattanDP.FrameId");

scoped_ptr<ManhattanReconstruction> recon;

ofstream out;

double sum_accuracy;
int num_frames;

void DoVaryGridSize(const Map& map,
                    const KeyFrame& kf,
                    const PosedImage& pim,
                    const proto::TruthedMap& tru_map,
                    const proto::TruthedFrame& tru_frame,
                    const MatI& gt_orients) {
	if (!out.is_open()) {
		out.open("scale_factor.dat");
		out << "# scale_factor  acc%  time\n";
	}

	for (int k = 1; k <= 32; k *= 2) {
		recon->dp.grid_scale_factor = k;
		WITHOUT_DLOG recon->Compute(pim, tru_map);
		double acc = GetAccuracy(recon->dp.soln_orients, gt_orients);

		out << recon->dp.grid_scale_factor << " " << acc << " "
		        << recon->dp.solve_time;
		TITLED("k="+lexical_cast<string>(k))
		{
			DREPORT(recon->dp.solve_time);
			DREPORT(acc);
		}
	}
}

void DoVaryJumpThresh(const Map& map, const KeyFrame& kf,
                      const PosedImage& pim, const proto::TruthedMap& tru_map,
                      const proto::TruthedFrame& tru_frame,
                      const MatI& gt_orients) {
	const int kSamples = 5;

	if (!out.is_open()) {
		out.open("jump_thresh.dat");
		out << "# jump_thresh  rel_acc%  time\n";
	}

	double zero_acc;
	for (int k = -1; k <= 2 * kSamples; k++) {
		recon->dp.jump_thresh = k < 0 ? 0
		        : pow(10.0, -2 + (1.0 * k / kSamples));
		WITHOUT_DLOG recon->Compute(pim, tru_map);
		double acc = GetAccuracy(recon->dp.soln_orients, gt_orients);
		if (k < 0) {
			zero_acc = acc;
		}
		double rel_acc = acc / zero_acc;

		out << recon->dp.jump_thresh << " " << rel_acc << " "
		        << recon->dp.solve_time << "\n";
		TITLED("k="+lexical_cast<string>(k))
		{
			DREPORT(recon->dp.jump_thresh);
			DREPORT(recon->dp.solve_time);
			DREPORT(rel_acc);
			DREPORT(recon->dp.full_backtrack.size());
		}
	}
}

void DoVaryK(const Map& map, const KeyFrame& kf, const PosedImage& pim,
             const proto::TruthedMap& tru_map,
             const proto::TruthedFrame& tru_frame, const MatI& gt_orients) {
	if (!out.is_open()) {
		out.open("max_corners.dat");
		out << "# max_corners   k   accuracy%\n";
	}

	for (int k = 1; k < 10; k++) {
		recon->dp.max_corners = k;
		WITHOUT_DLOG recon->Compute(pim, tru_map);
		double acc = GetAccuracy(recon->dp.soln_orients, gt_orients);

		out << k << " " << recon->dp.solve_time << " " << acc << "\n";
		TITLED("k="+lexical_cast<string>(k))
		{
			DREPORT(recon->dp.solve_time);
			DREPORT(acc);
		}
	}
	out.flush();
}

void DoViz(const Map& map, const KeyFrame& kf, const PosedImage& pim,
           const proto::TruthedMap& tru_map,
           const proto::TruthedFrame& tru_frame, const MatI& gt_orients) {
	int id = tru_frame.id();
	INDENTED TIMED("Reconstruction time") recon->Compute(pim, tru_map);
	double accuracy = GetAccuracy(recon->dp.soln_orients, gt_orients);
	sum_accuracy += accuracy;
	num_frames++;
	DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

	// Assemble the filepath
	format filepat("out/frame%03d_%s");

	// Write accuracy
	sofstream acc_out(str(filepat % id % "dp_accuracy.txt"));
	acc_out << static_cast<int>(accuracy*100) << endl;

	// Draw the floorplan estimated orientations
	ImageRGB<byte> initial_canvas;
	ImageCopy(kf.image.rgb, initial_canvas);
	DrawOrientations(recon->labeller.orient_map, initial_canvas, 0.5);
	WriteImage(str(filepat % id % "dp_initial.png"), initial_canvas);

	// Draw the predicted orientations
	ImageRGB<byte> soln_canvas;
	ImageCopy(kf.image.rgb, soln_canvas);
	DrawOrientations(recon->dp.soln_orients, soln_canvas, 0.5);
	WriteImage(str(filepat % id % "dp_soln.png"), soln_canvas);

	// Draw the solution in grid coordinates
	ImageRGB<byte> grid_canvas;
	recon->dp.DrawGridSolution(grid_canvas);
	WriteImage(str(filepat % id % "dp_grid.png"), grid_canvas);

	// Draw the ground truth orientations
	ImageRGB<byte> gt_canvas;
	ImageCopy(kf.image.rgb, gt_canvas);
	DrawOrientations(gt_orients, gt_canvas, 0.5);
	WriteImage(str(filepat % id % "gt.png"), gt_canvas);

	// Draw the original image
	WriteImage(str(filepat % id % "orig.png"), kf.image.rgb);
}

void ProcessFrame(int index, const Map& map, const proto::TruthedMap& tru_map) {
	// Pull out the truthed frame
	const proto::TruthedFrame& tru_frame = tru_map.frame(index);
	DLOG << format("Processing frame %d (index=%d of %d)") % tru_frame.id()
			        		% index % tru_map.frame_size();
	SCOPED_INDENT;

	// Pull out the frame
	const KeyFrame& kf = *map.KeyFrameById(tru_frame.id());
	if (&kf == NULL) {
		DLOG	<< "Warning: key frame " << tru_frame.id()
				        << " not found in the map";
		return;
	}

	// Construct the posed image
	PosedImage pim(*kf.pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	// Get ground truth orientations
	MatI gt_orients;
	LoadTrueOrients(tru_frame, gt_orients);
	InterchangeLabels(gt_orients, 0, 1);  // here we use a different convention for orient labels

	// Process
	DoViz(map, kf, pim, tru_map, tru_frame, gt_orients);
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG	<< "Usage: " << argv[0] << " truthed_map.pro";
		return 0;
	}

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(argv[1], ios::binary);
	CHECK(tru_map.ParseFromIstream(&s)) << "Failed to read from " << argv[1];

	// Load the map
	Map map;
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	// Initialize the reconstructor
	recon.reset(new ManhattanReconstruction);

	sum_accuracy = 0;
	num_frames = 0;

	// Process each frame
	for (int i = 0; i < tru_map.frame_size(); i++) {
		TIMED("Processing time") ProcessFrame(i, map, tru_map);
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG	<< format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	ofstream log_out("accuracies.txt", ios::out | ios::app);
	log_out << "DP " << (average_acc * 100) << "% " << argv[1] << endl;

	return 0;
}
