#include <iomanip>
#include <fstream>
#include <iterator>
#include <set>

#include <SVD.h>

#include "common_types.h"
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

#include "line_detector.h"
#include "vanishing_points.h"
#include "building_estimator.h"
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


void ProcessFrame(int index,
									const Map& map,
									const proto::TruthedMap& tru_map) {
	// Pull out the truthed frame
	const proto::TruthedFrame& tru_frame = tru_map.frame(index);
	int id = tru_frame.id();
	DLOG << format("Processing frame %d (index=%d of %d)")
		% tru_frame.id() % index % tru_map.frame_size();
	SCOPED_INDENT;

	// Pull out the frame
	const KeyFrame& kf = *map.KeyFrameById(id);
	if (&kf == NULL) {
		DLOG << "Error: key frame " << id << " not found in the map";
		return;
	}

	// Construct the posed image
	PosedCamera pc(kf.pc->pose, *map.camera);
	PosedImage pim(pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	// Detect lines
	GuidedLineDetector line_detector;
	TIMED("Detect lines") WITHOUT_DLOG line_detector.Compute(pim);

	// Compute line sweeps
	IsctGeomLabeller labeller;
	TIMED("Compute orientations") WITHOUT_DLOG labeller.Compute(pim, line_detector.detections);

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
	ManhattanRecovery recovery;
	recovery.Compute(pc, edges, labeller.orient_map);
	if (recovery.success) {
		DREPORT(recovery.soln_index);

		// Load ground truth. The branch--and--bound uses labels that
		// refer to the normal direction of the wall, which we indicate by
		// the third parameter below.
		MatI gt_orients;
		LoadTrueOrients(tru_frame, gt_orients, false);

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
									 kf.image, recovery.soln_orients);

		// Draw the predicted model in the other frames
		for (int refi = index+1; refi < index+5; refi++) {
			if (refi < tru_map.frame_size()) {
				const proto::TruthedFrame& ref_frame = tru_map.frame(refi);
				const KeyFrame& ref_kf = *map.KeyFrameByIdOrDie(ref_frame.id());
				TITLE("Projecting into frame " + lexical_cast<string>(ref_frame.id()));
				MatI ref_orients(ref_kf.image.ny(), ref_kf.image.nx());
				recovery.TransferBuilding(*recovery.soln,
																	kf.pc->pose,
																	ref_kf.pc->pose,
																	tru_map.floorplan().zceil(),
																	ref_orients,
																	map);
				
				WriteOrientViz(str(filepat % id % str(format("ref%03d.png")%ref_frame.id())),
											 ref_kf.image,
											 ref_orients);
			}
		}
	}
}






int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 2 && argc != 3) {
		DLOG << "Usage: "<<argv[0]<<" [FRAME_ID] truthed_map.pro";
		return 0;
	}

	int frame_id = -1;
	const char* tru_map_file;
	if (argc == 2) {
		tru_map_file = argv[1];
	} else {
		frame_id = atoi(argv[1]);
		tru_map_file = argv[2];
	}

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(tru_map_file, ios::binary);
	CHECK(tru_map.ParseFromIstream(&s)) << "Failed to read from " << tru_map_file;

	// Load the map
	Map map;
	map.auto_undistort = false;
	map.kf_ids_to_load.clear();
	BOOST_FOREACH(const proto::TruthedFrame& cur, tru_map.frame()) {
		map.kf_ids_to_load.push_back(cur.id());
	}
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	CHECK(GV3::get<int>("Map.LinearizeCamera") && map.camera != map.orig_camera)
		<< "Must use linear camera or multi-threaded structure estimation will fail";

	// TODO: map.Load() should do this automatically
	map.undistorter.Compute(ImageRef(640,480));

	vector<int> frame_ids;
	ParseMultiRange(GV3::get<string>("ManhattanRecovery.Frames"), frame_ids);

	sum_accuracy = 0;
	num_frames = 0;
	for (int i = 0; i < tru_map.frame_size(); i++) {
		if (frame_id == -1 || tru_map.frame(i).id() == frame_id) {
			TIMED("Processing time") ProcessFrame(i, map, tru_map);
		}
	}

	double average_acc = sum_accuracy/num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc*100);

	ofstream log_out("accuracies.txt", ios::out|ios::app);
	log_out << "BnB " << (average_acc*100) << "% " << tru_map_file << endl;

	return 0;
}
