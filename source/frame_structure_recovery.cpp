#include <iomanip>
#include <fstream>
#include <iterator>
#include <set>

#include <SVD.h>

#include <VW/Utils/timer.h>

#include "common_types_entry.h"
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
#include "canvas.h"

#include "line_detector.h"
#include "vanishing_points.h"
#include "building_estimator.h"
#include "bld_helpers.h"

#include "vector_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

lazyvar<int> gvOrientRes("ManhattanRecovery.OrientRes");

//ofstream datafile;

double sum_accuracy;
int num_frames;

void ProcessFrame(int index,
									Map& map,
									const proto::TruthedMap& tru_map,
									const vector<int>& nbr_indices) {
	// Pull out the truthed frame
	const proto::TruthedFrame& tru_frame = tru_map.frame(index);
	int id = tru_frame.id();
	DLOG << format("Processing frame %d (index=%d of %d)")
		% tru_frame.id() % index % tru_map.frame_size();
	SCOPED_INDENT;

	// Load ground truth. The branch--and--bound uses labels that
	// refer to the normal direction of the wall, which we indicate by
	// the third parameter below.
	MatI gt_orients;
	LoadTrueOrients(tru_frame, gt_orients, false);

	// Pull out the frame
	KeyFrame& kf = *map.KeyFrameById(tru_frame.id());
	if (&kf == NULL) {
		DLOG << "Warning: key frame " << tru_frame.id() << " not found in the map";
		return;
		}
	kf.LoadImage();

	// Construct the posed image
	PosedImage pim(*kf.pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	// Detect lines
	GuidedLineDetector line_detector;
	TIMED("Detect lines") WITHOUT_DLOG line_detector.Compute(pim);

	// Compute line sweeps
	IsctGeomLabeller labeller;
	TIMED("Compute orientations") WITHOUT_DLOG
		labeller.Compute(pim, line_detector.detections);

	// Copy the lines into structure recovery format
	vector<ManhattanEdge> edges[3];
	int next_id = 0;
	for (int i = 0; i < 3; i++) {
		COUNTED_FOREACH(int j, const LineDetection& det, line_detector.detections[i]) {
			Vector<3> a = map.camera->ImToRet(det.seg.start);
			Vector<3> b = map.camera->ImToRet(det.seg.end);
			edges[i].push_back(ManhattanEdge(a, b, i, next_id++));
		}
	}
	DLOG << format("Line counts by axis: %d, %d, %d")
		% edges[0].size() % edges[1].size() % edges[2].size();
	int num_lines = edges[0].size() + edges[1].size() + edges[2].size();

	// Enumerate building hypotheses
	VW::Timer recovery_timer;
	ManhattanRecovery recovery;
	recovery.Compute(*kf.pc, edges, labeller.orient_map);
	double ms = recovery_timer.GetAsMilliseconds();
	
	if (recovery.success) {
		//		datafile << format("%d %d %f\n") % tru_frame.id() % num_lines % ms;

		// Compute accuracy
		double accuracy = GetAccuracy(recovery.soln_orients, gt_orients);
		sum_accuracy += accuracy;
		num_frames++;
		DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

		// Write accuracy
		format filepat("out/frame%03d_%s");
		sofstream acc_out(str(filepat % id % "bnb_accuracy.txt"));
		acc_out << static_cast<int>(accuracy*100) << endl;

		// Draw the original image
		WriteImage(str(filepat % id % "orig.png"), kf.image.rgb);

		// Draw lines and line sweeps
		ImageRGB<byte> orient_canvas;
		ImageCopy(pim.rgb, orient_canvas);
		labeller.DrawOrientViz(orient_canvas);
		line_detector.DrawSegments(orient_canvas);
		WriteImage(str(filepat % id % "bnb_orients.png"), orient_canvas);

		// Draw the line detections
		FileCanvas line_canvas(str(filepat % id % "bnb_lines.png"), asToon(kf.image.sz()));
		line_canvas.DrawImage(kf.image.rgb);
		line_canvas.SetLineWidth(3.0);
		for (int i = 0; i < 3; i++) {
			BOOST_FOREACH(const LineDetection& det, line_detector.detections[i]) {
				line_canvas.StrokeLine(det.seg, Colors::primary(i));
			}
		}

		// Draw the predicted model
		ImageRGB<byte> soln_canvas;
		ImageCopy(kf.image.rgb, soln_canvas);
		DrawOrientations(recovery.soln_orients, soln_canvas, 0.35);
		WriteImage(str(filepat % id % "bnb_soln.png"), soln_canvas);

		// Draw the prediction in neighbouring frames
		MatI nbr_prediction(kf.image.ny(), kf.image.nx());
		ImageRGB<byte> nbr_canvas;
		BOOST_FOREACH(int nbr_id, nbr_indices) {
			if (nbr_id == id) continue;
			DLOG << "Drawing neighbour: frame " << nbr_id;
			KeyFrame& nbr = map.kfs[nbr_id];
			nbr.LoadImage();
			recovery.TransferBuilding(recovery.soln, kf.pc->pose, nbr.pc->pose,
																tru_map.floorplan().zfloor(), nbr_prediction);
			ImageCopy(nbr.image.rgb, nbr_canvas);
			DrawOrientations(nbr_prediction, nbr_canvas, 0.35);
			string file_label = str(format("nbr_%03d.png") % nbr_id);
			WriteImage(str(filepat % id % file_label), nbr_canvas);
			nbr.UnloadImage();
		}
	}

	kf.UnloadImage();
}






int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc < 2 || argc > 4) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro INDEX NBRS";
		return 0;
	}

	int kf_index = -1;
	if (argc >= 3) {
		kf_index = atoi(argv[2]);
	}

	vector<int> nbr_indices;
	if (argc >= 4) {
		ParseMultiRange(argv[3], nbr_indices);
	}

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(argv[1], ios::binary);
	CHECK(tru_map.ParseFromIstream(&s)) << "Failed to read from " << argv[1];

	// Load the map
	Map map;
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	CHECK(GV3::get<int>("Map.LinearizeCamera") && map.camera != map.orig_camera)
		<< "Must use linear camera or multi-threaded structure estimation will fail";

	// TODO: map.Load() should do this automatically
	map.undistorter.Compute(ImageRef(640,480));

	vector<int> frame_ids;
	ParseMultiRange(GV3::get<string>("ManhattanRecovery.Frames"), frame_ids);

	/*datafile.open("bnb_timings.dat");
		datafile << "Keyframe-id Line-detections Time(ms)\n";*/

	sum_accuracy = 0;
	num_frames = 0;
	for (int i = 0; i < tru_map.frame_size(); i++) {
		if (tru_map.frame(i).id() != kf_index && kf_index != -1) continue;
		TIMED("Processing time") ProcessFrame(i, map, tru_map, nbr_indices);
	}

	double average_acc = sum_accuracy/num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc*100);

	ofstream log_out("accuracies.txt", ios::out|ios::app);
	log_out << "BnB " << (average_acc*100) << "% " << argv[1] << endl;

	return 0;
}
