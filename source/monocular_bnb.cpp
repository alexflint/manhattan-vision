#include <iomanip>
#include <fstream>
#include <iterator>
#include <set>

#include <SVD.h>
#include <boost/filesystem.hpp>

#include <VW/Utils/timer.h>

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
#include "canvas.h"

#include "line_detector.h"
#include "vanishing_points.h"
#include "manhattan_bnb.h"
#include "bld_helpers.h"

#include "counted_foreach.tpp"
#include "vector_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

lazyvar<int> gvOrientRes("ManhattanRecovery.OrientRes");

//ofstream datafile;

class ManhattanBnbReconstructor {
public:
	const Frame* input;
	GuidedLineDetector line_detector;
	IsctGeomLabeller line_sweeper;
	MonocularManhattanBnb manhattan_bnb;
	bool success;
	double bnb_time_ms;

	// Run the reconstruction algorithm for the given frame
	void Compute(const Frame& frame);

	// Compute accuracy w.r.t. ground truth
	double GetAccuracy(const MatI& gt_orients);
	// Compute accuracy w.r.t. the ground truth floorplan
	double GetAccuracy(const proto::FloorPlan& gt_floorplan);

	// Draw the original image
	void OutputOrigViz(const string& path);
	// Draw lines and line sweeps
	void OutputOrientViz(const string& path);
	// Draw the line detections
	void OutputLineViz(const string& path);
	// Draw the predicted model
	void OutputSolutionOrients(const string& path);
	// Transfer the solution to an auxilliary view
	void GetAuxOrients(const PosedCamera& aux, double zfloor, MatI& aux_orients);
	// Draw the solution in an auxilliary view
	void OutputSolutionInView(const string& path,
	                          const Frame& aux,
	                          double zfloor);
};


void ManhattanBnbReconstructor::Compute(const Frame& frame) {
	input = &frame;
	CHECK(frame.image.loaded()) << "Frame must have its image loaded";

	// Construct the posed image
	PosedImage pim(*frame.pc);
	ImageCopy(frame.image.rgb, pim.rgb);

	// Detect lines
	TIMED("Detect lines") line_detector.Compute(pim);

	// Compute line sweeps
	TIMED("Compute orientations") line_sweeper.Compute(pim, line_detector.detections);

	// Copy the lines into structure recovery format
	vector<ManhattanEdge> edges[3];
	int next_id = 0;
	for (int i = 0; i < 3; i++) {
		COUNTED_FOREACH(int j, const LineDetection& det, line_detector.detections[i]) {
			Vec3 a = frame.pc->ImToRet(det.seg.start);
			Vec3 b = frame.pc->ImToRet(det.seg.end);
			edges[i].push_back(ManhattanEdge(a, b, i, next_id++));
		}
	}
	DLOG << format("Line counts by axis: %d, %d, %d")
						% edges[0].size() % edges[1].size() % edges[2].size();

	// Enumerate building hypotheses
	VW::Timer recovery_timer;
	success = manhattan_bnb.Compute(*frame.pc, edges, line_sweeper.orient_map);
	bnb_time_ms = recovery_timer.GetAsMilliseconds();
}

double ManhattanBnbReconstructor::GetAccuracy(const MatI& gt_orients) {
	return ComputeAgreementPct(manhattan_bnb.soln_orients, gt_orients);
}

double ManhattanBnbReconstructor::GetAccuracy(const proto::FloorPlan& gt_floorplan) {
	MatI gt_orients;
	GetTrueOrients(gt_floorplan, *input->pc, gt_orients);
	return GetAccuracy(gt_orients);
}

void ManhattanBnbReconstructor::OutputOrigViz(const string& path) {
	WriteImage(path, input->image.rgb);
}

void ManhattanBnbReconstructor::OutputOrientViz(const string& path) {
	ImageRGB<byte> orient_canvas;
	ImageCopy(input->image.rgb, orient_canvas);
	line_sweeper.DrawOrientViz(orient_canvas);
	line_detector.DrawSegments(orient_canvas);
	WriteImage(path, orient_canvas);
}

void ManhattanBnbReconstructor::OutputLineViz(const string& path) {
	FileCanvas line_canvas(path, asToon(input->image.sz()));
	line_canvas.DrawImage(input->image.rgb);
	line_canvas.SetLineWidth(3.0);
	for (int i = 0; i < 3; i++) {
		BOOST_FOREACH(const LineDetection& det, line_detector.detections[i]) {
			line_canvas.StrokeLine(det.seg, Colors::primary(i));
		}
	}
}

void ManhattanBnbReconstructor::OutputSolutionOrients(const string& path) {
	ImageRGB<byte> soln_canvas;
	ImageCopy(input->image.rgb, soln_canvas);
	DrawOrientations(manhattan_bnb.soln_orients, soln_canvas, 0.35);
	WriteImage(path, soln_canvas);
}

void ManhattanBnbReconstructor::GetAuxOrients(const PosedCamera& aux,
                                              double zfloor,
                                              MatI& aux_orients) {
	aux_orients.Resize(aux.image_size().y, aux.image_size().x);
	manhattan_bnb.TransferBuilding(manhattan_bnb.soln,
																 aux.pose(),
																 zfloor,
																 aux_orients);
}

void ManhattanBnbReconstructor::OutputSolutionInView(const string& path,
                                                     const Frame& aux,
                                                     double zfloor) {
	// Generate the orientation map
	MatI aux_orients;
	GetAuxOrients(aux.image.pc(), zfloor, aux_orients);
	CHECK(aux.image.loaded()) << "Auxiliary view not loaded";

	// Draw the image
	ImageRGB<byte> aux_canvas;
	ImageCopy(aux.image.rgb, aux_canvas);
	DrawOrientations(aux_orients, aux_canvas, 0.35);
	WriteImage(path, aux_canvas);
}







double sum_accuracy;
int num_frames;

ManhattanBnbReconstructor reconstructor;

void ProcessFrame(Map& map,
                  const proto::TruthedMap& tru_map,
                  int frame_id,
                  const vector<int>& aux_ids) {
	// Run the reconstruction
	KeyFrame& kf = *map.KeyFrameByIdOrDie(frame_id);
	kf.LoadImage();
	reconstructor.Compute(kf);

	if (reconstructor.success) {
		// Compute accuracy
		double accuracy = reconstructor.GetAccuracy(tru_map.floorplan());
		sum_accuracy += accuracy;
		num_frames++;
		DLOG << format("Accuracy: %.2f%%") % (accuracy*100);

		// Write accuracy
		format filepat("out/frame%03d_%s");
		sofstream acc_out(str(filepat % frame_id % "bnb_accuracy.txt"));
		acc_out << static_cast<int>(accuracy*100) << endl;

		// Output
		reconstructor.OutputOrigViz(str(filepat % frame_id % "orig.png"));
		reconstructor.OutputOrientViz(str(filepat % frame_id % "bnb_orients.png"));
		reconstructor.OutputLineViz(str(filepat % frame_id % "bnb_lines.png"));
		reconstructor.OutputSolutionOrients(str(filepat % frame_id % "bnb_soln.png"));

		// Draw the prediction in neighbouring frames
		BOOST_FOREACH(int aux_id, aux_ids) {
			if (aux_id == frame_id) continue;
			DLOG << "Drawing auxiliary frame " << aux_id;
			string file_label = str(format("nbr_%03d.png") % aux_id);
			KeyFrame* aux_frame = map.KeyFrameById(aux_id);
			if (aux_frame) {
				aux_frame->LoadImage();
				reconstructor.OutputSolutionInView(str(filepat % aux_id % file_label),
						*aux_frame, tru_map.floorplan().zfloor());
				aux_frame->UnloadImage();
			}
		}
	}

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

	// Load the map
	Map map;
	proto::TruthedMap tru_map;
	map.LoadWithGroundTruth(argv[1], tru_map);

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
	log_out << "BnB " << (average_acc * 100) << "% " << argv[1] << endl;

	return 0;
}
