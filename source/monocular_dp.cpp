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
scoped_ptr<ManhattanDPReconstructor> recon;

double sum_accuracy;
int num_frames;

void ProcessFrame(Map& map, const proto::TruthedMap& tru_map, int frame_id) {
	// Pull out the frame
	KeyFrame& kf = *map.KeyFrameByIdOrDie(frame_id);
	kf.LoadImage();

	// Construct the posed image
	PosedImage pim(*kf.pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	// Perform the reconstruction
	INDENTED TIMED("Reconstruction time") recon->Compute(pim, tru_map);
	format filepat("out/frame%03d_%s");

	// Load ground truth
	MatI gt_orients;
	GetTrueOrients(tru_map.floorplan(), *kf.pc, gt_orients);

	// Compute accuracy
	double accuracy = recon->GetAccuracy(gt_orients);
	sum_accuracy += accuracy;
	num_frames++;
	DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

	// Draw some vizualizations
	WriteOrientationImage(str(filepat % frame_id % "gt.png"), gt_orients);
	recon->OutputOrigViz(str(filepat % frame_id % "orig.png"));
	recon->OutputOrientViz(str(filepat % frame_id % "dp_initial.png"));
	recon->OutputSolutionOrients(str(filepat % frame_id % "dp_soln.png"));
	//recon->OutputOppRowViz(str(filepat % frame_id % "opprows.png"));

	kf.UnloadImage();
}

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc < 2 || argc > 3) {
		DLOG	<< "Usage: " << argv[0] << " truthed_map.pro [INDEX]";
		return 0;
	}

	int kf_index = -1;
	if (argc >= 3) {
		kf_index = atoi(argv[2]);
	}

	// Load the map
	Map map;
	proto::TruthedMap tru_map;
	map.LoadWithGroundTruth(argv[1], tru_map);

	// Set up the reconstruction
	recon.reset(new ManhattanDPReconstructor);

	sum_accuracy = 0;
	num_frames = 0;
	if (kf_index == -1) {
		for (int i = 0; i < map.kfs.size(); i++) {
			DLOG << format("Processing frame %d of %d") % i % map.kfs.size();
			INDENTED TIMED("Processing time") ProcessFrame(map, tru_map, i);
		}
	} else {
		TIMED("Processing time") ProcessFrame(map, tru_map, kf_index);
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	ofstream log_out("accuracies.txt", ios::out | ios::app);  // Output for appending
	log_out << "DP " << (average_acc * 100) << "% " << argv[1] << endl;

	return 0;
}
