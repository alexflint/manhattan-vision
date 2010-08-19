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

// Pass the ground truth orientations as the initial orientations
// Use this for testing the algorithm
bool pass_gt_orients = false;

// To be moved elsewhere?
scoped_ptr<ManhattanDPReconstructor> recon;

double sum_accuracy;
int num_frames;

void ProcessFrame(Map& map, const proto::TruthedMap& gt_map, int frame_id) {
	// Pull out the frame
	KeyFrame& kf = *map.KeyFrameByIdOrDie(frame_id);
	kf.LoadImage();

	// Construct the posed image
	PosedImage pim(*kf.pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	// Perform the reconstruction
	INDENTED TIMED("Reconstruction time") {
		recon->Compute(pim, gt_map, pass_gt_orients);
	}
	format filepat("out/frame%03d_%s");

	// Compute accuracy
	double accuracy = recon->GetAccuracy(recon->gt_orients);
	sum_accuracy += accuracy;
	num_frames++;
	DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

	// Draw some vizualizations
	WriteOrientationImage(str(filepat % frame_id % "gt.png"), recon->gt_orients);
	recon->OutputOrigViz(str(filepat % frame_id % "orig.png"));
	recon->OutputOrientViz(str(filepat % frame_id % "dp_initial.png"));
	recon->OutputSolutionOrients(str(filepat % frame_id % "dp_soln.png"));
	//recon->OutputOppRowViz(str(filepat % frame_id % "opprows.png"));

	kf.UnloadImage();
}

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc < 2 || argc > 4) {
		DLOG << "Usage: " << argv[0] << " truthed_map.pro [INDEX] [--from_gt]";
		return 0;
	}

	vector<int> indices;
	if (argc > 2) indices = ParseMultiRange<int>(argv[2]);

	if (argc >= 4 && string(argv[3]) == "--from_gt") {
		pass_gt_orients = true;
	} else if (argc >= 4) {
		DLOG << "Usage: " << argv[0] << " truthed_map.pro [INDEX] [--from_gt]";
		return 0;
	}

	// Load the map
	Map map;
	proto::TruthedMap tru_map;
	map.LoadWithGroundTruth(argv[1], tru_map);

	// Set up the reconstruction
	recon.reset(new ManhattanDPReconstructor);

	sum_accuracy = 0;
	num_frames = 0;
	if (indices.empty()) {
		for (int i = 0; i < map.kfs.size(); i++) {
			DLOG << format("Processing frame %d of %d") % i % map.kfs.size();
			INDENTED TIMED("Processing time") ProcessFrame(map, tru_map, i);
		}
	} else {
		BOOST_FOREACH(int index, indices) {
			TIMED("Processing time") ProcessFrame(map, tru_map, index);
		}
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	ofstream log_out("accuracies.txt", ios::out | ios::app);  // Output for appending
	log_out << "DP " << (average_acc * 100) << "% " << argv[1] << endl;

	return 0;
}
