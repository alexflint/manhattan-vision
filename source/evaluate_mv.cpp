#include <set>

#include <boost/array.hpp>
#include <boost/ptr_container/ptr_map.hpp>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"

#include "camera.h"
#include "colors.h"
#include "canvas.h"
#include "bld_helpers.h"
#include "manhattan_dp.h"
#include "multiview_reconstructor.h"
#include "timer.h"

#include "io_utils.tpp"
#include "integral_col_image.tpp"
#include "vector_utils.tpp"
#include "math_utils.tpp"

using std::set;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 4) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro TEST_IDS NUM_AUX_VIEWS";
		return -1;
	}

	// Input arguments
	const char* path = argv[1];
	const vector<int> test_ids = ParseMultiRange<int>(argv[2]);
	const int num_aux_views = atoi(argv[3]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(path, gt_map);

	// Get the floor and ceiling positions
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();
	Vec3 vup = map.kfs[0].pc->pose_inverse() * makeVector(0,1,0);
	if (Sign(zceil-zfloor) == Sign(vup[2])) {
		swap(zfloor, zceil);
	}

	// Calculate all the frames we need to compute line sweeps for
	set<int> include_ids;
	BOOST_FOREACH(int id, test_ids) {
		for (int j = max(0, id-num_aux_views); j <= min(map.kfs.size()-1, id+num_aux_views); j++) {
			include_ids.insert(j);
		}
	}

	// Compute all the objective functions
	ptr_map<int,DPObjective> objectives;
	LineSweepDPScore gen;
	BOOST_FOREACH(int id, include_ids) {
		Frame* frame = map.KeyFrameByIdOrDie(id);
		frame->LoadImage();
		DLOG << "Computing objective function for frame " << id;
		INDENTED TIMED("Compute objectives") INDENTED {
			gen.Compute(frame->image);
			objectives.insert(id, new DPObjective);
			gen.score_func.CopyTo(objectives[id]);  // ugh, copying large matrices :(
		}
	}

	double sum_acc_mono = 0, sum_acc_mv = 0;

	// Do the actual evaluation
	format filepat("out/frame%03d_%s.png");
	MultiViewReconstructor mv;
	ManhattanDPReconstructor mono;
	BOOST_FOREACH(int base_id, test_ids) {
		TITLE(str(format("Reconstructing frame %d")%base_id));

		// Set up the reconstruction

		// Add the base frame
		Frame* base_frame = map.KeyFrameByIdOrDie(base_id);
		base_frame->LoadImage();
		mv.Configure(base_frame->image, objectives[base_id], zfloor, zceil);

		// Add the auxiliary frames
		DLOG_N << "Using auxiliary frames: ";
		for (int aux_id = max(0, base_id-num_aux_views);
				 aux_id <= min(map.kfs.size()-1, base_id+num_aux_views);
				 aux_id++) {
			if (aux_id == base_id) continue;

			DLOG_N << aux_id << " ";
			Frame* frame = map.KeyFrameByIdOrDie(aux_id);
			frame->LoadImage();
			mv.AddFrame(frame->image, objectives[aux_id]);
		}
		DLOG << endl;

		// Joint reconstruction
		TITLED("Doing joint reconstruction")
			mv.Reconstruct();

		// Monocular reconstruction
		Mat3 fToC = GetManhattanHomology(base_frame->image.pc(), zfloor, zceil);
		TITLED("Doing monocular reconstruction")
			mono.Compute(base_frame->image, fToC, objectives[base_id]);

		// Report accuracy
		MatI gt_orients;
		GetTrueOrients(gt_map.floorplan(), base_frame->image.pc(), gt_orients);
		double acc_mono = mono.GetAccuracy(gt_orients);
		double acc_mv = mv.recon.GetAccuracy(gt_orients);
		sum_acc_mono += acc_mono;
		sum_acc_mv += acc_mv;
		DLOG << format("Accuracy (monocular):  %.2f%%") % (acc_mono*100);
		DLOG << format("Accuracy (multiview):  %.2f%%") % (acc_mv*100);

		// Produce visualisations
		mv.recon.OutputSolutionOrients(str(filepat % base_id % "mv_soln.png"));
		mono.OutputSolutionOrients(str(filepat % base_id % "mono_soln.png"));

		/*mv.base_objective.line_sweeper.OutputOrientViz("out/base_sweeps.png");
		WriteMatrixImageRescaled("out/base_raw_L.png", mv.joint_payoffs.base_payoffs[0]);
		WriteMatrixImageRescaled("out/base_raw_R.png", mv.joint_payoffs.base_payoffs[1]);
		WriteMatrixImageRescaled("out/joint_raw_L.png", mv.joint_payoffs.payoffs[0]);
		WriteMatrixImageRescaled("out/joint_raw_R.png", mv.joint_payoffs.payoffs[1]);
		mv.OutputBasePayoffs("out/base_payoffs.png");
		mv.OutputAuxPayoffs("out/");
		mv.OutputJointPayoffs("out/joint_payoffs.png");
		OutputPayoffsViz("out/mono_payoffs.png",
										 mv.base_frame->rgb,
										 mono.dp.payoffs,
										 *mono.dp.geom);*/
	}

	double av_acc_mono = sum_acc_mono / test_ids.size();
	double av_acc_mv = sum_acc_mv / test_ids.size();
	DLOG << endl;
	DLOG << format("Average Accuracy (monocular):  %.2f%%") % (av_acc_mono*100);
	DLOG << format("Average Accuracy (multiview):  %.2f%%") % (av_acc_mv*100);

	return 0;
}
