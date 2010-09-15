#include <iomanip>
#include <fstream>
#include <iterator>
#include <set>

#include <SVD.h>

#include "entrypoint_types.h"
#include "map.h"
#include "vars.h"
#include "line_sweeper.h"
#include "timer.h"
#include "geom_utils.h"
#include "worker.h"
#include "clipping.h"
#include "viewer3d.h"
#include "guided_line_detector.h"
#include "map_widgets.h"
#include "progress_reporter.h"
#include "safe_stream.h"
#include "manhattan_bnb.h"
#include "line_detector.h"
#include "vanishing_points.h"
#include "bld_helpers.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

lazyvar<int> gvOrientRes("ManhattanRecovery.OrientRes");

ofstream datafile;

double sum_accuracy;
int num_frames;

void WriteOrientViz(const string& filename,
                    const ImageBundle& image,
                    const MatI& orients) {
	ImageRGB<byte> canvas;
	ImageCopy(image.rgb, canvas);
	DrawOrientations(orients, canvas, 0.5);
	WriteImage(filename, canvas);
}

void ProcessFrame(int id, const Map& map, const proto::TruthedMap& tru_map) {
	// Pull out the truthed frame
	DLOG << format("Processing frame %d of %d") % id % map.kfs.size();
	SCOPED_INDENT;

	// Pull out the frame
	const KeyFrame& kf = *map.KeyFrameById(id);
	if (&kf == NULL) {
		DLOG	<< "Error: key frame " << id << " not found in the map";
		return;
	}

	// Detect lines
	GuidedLineDetector line_detector;
	TIMED("Detect lines")
		WITHOUT_DLOG line_detector.Compute(kf.image);
	// Compute line sweeps
	IsctGeomLabeller labeller;
	TIMED("Compute orientations")
		WITHOUT_DLOG labeller.Compute(kf.image, line_detector.detections);

	// Copy the lines into structure recovery format
	vector<ManhattanEdge> edges[3];
	int next_id = 0;
	for (int i = 0; i < 3; i++) {
		COUNTED_FOREACH(int j, const LineDetection& det, line_detector.detections[i]) {
			Vector<3> a = unproject(map.camera->ImToRet(project(det.seg.start)));
			Vector<3> b = unproject(map.camera->ImToRet(project(det.seg.end)));
			edges[i].push_back(ManhattanEdge(a, b, i, next_id++));
		}
	}
	DLOG << format("Line counts by axis: %d, %d, %d")
	% edges[0].size() % edges[1].size() % edges[2].size();

	// Enumerate building hypotheses
	MonocularManhattanBnb recovery;
	bool success = recovery.Compute(pc, edges, labeller.orient_map);
	if (success) {
		// Load ground truth. The branch--and--bound uses labels that
		// refer to the normal direction of the wall, which we indicate by
		// the third parameter below.
		MatI gt_orients;
		GetTrueOrients(tru_map.floorplan(), *kf.pc, gt_orients);

		// Compute accuracy
		double accuracy = GetAccuracy(recovery.soln_orients, gt_orients);
		sum_accuracy += accuracy;
		num_frames++;
		DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

		// Write accuracy
		format filepat("out/frame%03d_%s");
		sofstream acc_out(str(filepat % id % "bnb_accuracy.txt"));
		acc_out << static_cast<int>(accuracy*100) << endl;

		// Vizualize
		ImageRGB<byte> orient_canvas;
		ImageCopy(pim.rgb, orient_canvas);
		labeller.DrawOrientViz(orient_canvas);
		line_detector.DrawSegments(orient_canvas);
		WriteImage(str(filepat % id % "bnb_orients.png"), orient_canvas);
		line_detector.OutputSegmentsViz(str(filepat % id % "bnb_lines.png"));

		// Draw the predicted model
		WriteOrientViz(str(filepat % id % "bnb_soln.png"),
					   kf.image,
					   recovery.soln_orients);

		// Draw the predicted model in the other frames
		for (int ref_id = id+1; ref_id < id+5; ref_id++) {
			if (ref_id < map.kfs.size()) {
				const KeyFrame& ref_kf = *map.KeyFrameByIdOrDie(ref_id);
				TITLE("Projecting into frame " + lexical_cast<string>(ref_id));
				MatI ref_orients(ref_kf.image.ny(), ref_kf.image.nx());
				recovery.TransferBuilding(recovery.soln,
						kf.pc->pose,
						ref_kf.pc->pose,
						tru_map.floorplan().zceil(),
						ref_orients);

				WriteOrientViz(str(filepat % id % str(format("ref%03d.png")%ref_id)),
							   ref_kf.image,
							   ref_orients);
			}
		}
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2 && argc != 3) {
		cout << "Usage: "<<argv[0]<<" truthed_map.pro [INDEX]" << endl;
		return 0;
	}

	const char* tru_map_file = argv[1];
	int frame_id = (argc == 3) ?  atoi(argv[2]) : -1;

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(tru_map_file, ios::binary);
	CHECK(tru_map.ParseFromIstream(&s)) << "Failed to read from " << tru_map_file;

	// Load the map
	Map map;
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	sum_accuracy = 0;
	num_frames = 0;
	if (frame_id == -1) {
		for (int i = 0; i < tru_map.frame_size(); i++) {
			TIMED("Processing time") ProcessFrame(i, map, tru_map);
		}
	} else {
		TIMED("Processing time") ProcessFrame(frame_id, map, tru_map);
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	ofstream log_out("accuracies.txt", ios::out | ios::app);
	log_out << "BnB " << (average_acc * 100) << "% " << tru_map_file << endl;

	return 0;
}
