#include <boost/array.hpp>

#include <LU.h>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"

#include "camera.h"
#include "colors.h"
#include "canvas.h"
#include "bld_helpers.h"
#include "manhattan_dp.h"
#include "multiview_reconstructor.h"

#include "io_utils.tpp"
#include "integral_col_image.tpp"
#include "vector_utils.tpp"
//#include "numeric_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 4) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro BASE_INDEX AUX_RANGE";
		return 0;
	}

	// Input arguments
	const char* path = argv[1];
	int base_id = atoi(argv[2]);
	vector<int> aux_ids = ParseMultiRange<int>(string(argv[3]));
	// Never use the base frame as an auxiliary frame
	vector<int>::iterator new_end = remove(aux_ids.begin(), aux_ids.end(), base_id);
	aux_ids.erase(new_end, aux_ids.end());

	format filepat("out/%s");
	DREPORT(base_id, iowrap(aux_ids));

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

	// Set up the reconstruction
	MultiViewReconstructor mv;
	Frame* base_frame = map.KeyFrameByIdOrDie(base_id);
	base_frame->LoadImage();
	TITLED("Computing objective function for base frame")
		mv.Configure(base_frame->image, zfloor, zceil);

	BOOST_FOREACH(int aux_id, aux_ids) {
		Frame* frame = map.KeyFrameByIdOrDie(aux_id);
		frame->LoadImage();
		TITLED(str(format("Computing objective function for frame %d")%aux_id))
			mv.AddFrame(frame->image);
	}
	TITLED("Doing joint reconstruction")
		mv.Reconstruct();

	// Now do the monocular reconstruction
	LineSweepDPScore gen(*base_frame);
	ManhattanDPReconstructor mono;
	TITLED("Doing monocular reconstruction")
		mono.Compute(*mv.base_frame,
								 mv.joint_payoffs.base_geom.floorToCeil,
								 gen.objective);

	// Report accuracy
	MatI gt_orients;
	GetTrueOrients(gt_map.floorplan(), base_frame->image.pc(), gt_orients);
	double mono_accuracy = mono.GetAccuracy(gt_orients);
	double mv_accuracy = mv.recon.GetAccuracy(gt_orients);
	DLOG << format("Accuracy (monocular):  %.2f%%") % (mono_accuracy*100);
	DLOG << format("Accuracy (multiview):  %.2f%%") % (mv_accuracy*100);

	// Produce visualisations
	mv.recon.OutputSolutionOrients(str(filepat % "mv_soln.png"));
	mono.OutputSolutionOrients(str(filepat % "mono_soln.png"));

	DLOG << "Done outputting solutions";

	mv.base_gen.line_sweeper.OutputOrientViz("out/base_sweeps.png");

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
									 *mono.dp.geom);
	return 0;
}
