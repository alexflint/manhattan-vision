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

#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

// To be moved elsewhere?
scoped_ptr<ManhattanDPReconstruction> recon;

double sum_accuracy;
int num_frames;

void ProcessFrame(Map& map,
                  const proto::TruthedMap& tru_map,
                  int frame_id,
                  const vector<int>& aux_ids) {
	// Pull out the frame
	KeyFrame& kf = *map.KeyFrameByIdOrDie(frame_id);
	kf.LoadImage();

	// Construct the posed image
	PosedImage pim(*kf.pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	// Get ground truth orientations
	MatI gt_orients;
	GetTrueOrients(tru_map.floorplan(), *kf.pc, gt_orients);
	InterchangeLabels(gt_orients, 0, 1);  // here we use a different convention for orient labels

	// Perform the reconstruction
	INDENTED TIMED("Reconstruction time") recon->Compute(pim, tru_map);
	format filepat("out/frame%03d_%s");

	// Compute accuracy
	double accuracy = ComputeAgreementPct(recon->dp.soln_orients, gt_orients);
	sum_accuracy += accuracy;
	num_frames++;
	DLOG << format("Accuracy: %.2f%%") % (accuracy*100);
	sofstream acc_out(str(filepat % frame_id % "dp_accuracy.txt"));
	acc_out << static_cast<int>(accuracy*100) << endl;

	// Draw the original image
	WriteImage(str(filepat % frame_id % "orig.png"), kf.image.rgb);

	// Draw the floorplan estimated orientations
	ImageRGB<byte> initial_canvas;
	ImageCopy(kf.image.rgb, initial_canvas);
	DrawOrientations(recon->labeller.orient_map, initial_canvas, 0.5);
	WriteImage(str(filepat % frame_id % "dp_initial.png"), initial_canvas);

	// Draw the predicted orientations
	ImageRGB<byte> soln_canvas;
	ImageCopy(kf.image.rgb, soln_canvas);
	DrawOrientations(recon->dp.soln_orients, soln_canvas, 0.5);
	WriteImage(str(filepat % frame_id % "dp_soln.png"), soln_canvas);

	// Draw the solution in grid coordinates
	ImageRGB<byte> grid_canvas;
	recon->dp.DrawGridSolution(grid_canvas);
	WriteImage(str(filepat % frame_id % "dp_grid.png"), grid_canvas);

	// Draw the ground truth orientations
	ImageRGB<byte> gt_canvas;
	ImageCopy(kf.image.rgb, gt_canvas);
	DrawOrientations(gt_orients, gt_canvas, 0.5);
	WriteImage(str(filepat % frame_id % "gt.png"), gt_canvas);

	kf.UnloadImage();
}

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc < 2 || argc > 4) {
		DLOG	<< "Usage: " << argv[0] << " truthed_map.pro INDEX NBRS";
		return 0;
	}

	int kf_index = -1;
	if (argc >= 3) {
		kf_index = atoi(argv[2]);
	}

	vector<int> aux_ids;
	if (argc >= 4) {
		ParseMultiRange(argv[3], aux_ids);
	}

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(argv[1], ios::binary);
	CHECK(tru_map.ParseFromIstream(&s)) << "Failed to read from " << argv[1];

	// Load the map
	Map map;
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	sum_accuracy = 0;
	num_frames = 0;
	if (kf_index == -1) {
		for (int i = 0; i < tru_map.frame_size(); i++) {
			DLOG << format("Processing frame %d of %d") % i % map.kfs.size();
			INDENTED TIMED("Processing time") ProcessFrame(map, tru_map, i, aux_ids);
		}
	} else {
		TIMED("Processing time") ProcessFrame(map, tru_map, kf_index, aux_ids);
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	ofstream log_out("accuracies.txt", ios::out | ios::app);
	log_out << "DP " << (average_acc * 100) << "% " << argv[1] << endl;

	return 0;
}
