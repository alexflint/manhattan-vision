#include <boost/array.hpp>

#include "entrypoint_types.h"
#include "map.h"
#include "colors.h"
#include "canvas.h"
#include "timer.h"
#include "manhattan_dp.h"
#include "stereo_payoffs.h"
#include "geom_utils.h"
#include "bld_helpers.h"
#include "floorplan_renderer.h"
#include "landmark_payoffs.h"

#include "io_utils.tpp"
#include "image_utils.tpp"
#include "numeric_utils.tpp"
#include "vector_utils.tpp"

using namespace toon;
using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG << "Usage: "<<argv[0]<<" SEQUENCE FRAME_IDS";
		return -1;
	}

	// Input arguments
	const char* sequence = argv[1];
	const vector<int> test_ids = ParseMultiRange<int>(argv[2]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	// Get the floor and ceiling positions
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();
	/*Vec3 vup = map.kfs[0].pc->pose_inverse() * makeVector(0,1,0);
	if (Sign(zceil-zfloor) == Sign(vup[2])) {
		swap(zfloor, zceil);
		gt_map.mutable_floorplan()->set_zfloor(zfloor);
		gt_map.mutable_floorplan()->set_zceil(zceil);
		}*/

	// Payoffs
	DPPayoffs payoffs;
	ManhattanDPReconstructor recon;

	// Reconstruct each frame
	double sum_acc = 0;
	format filepat("out/frame%02d_%s.png");
	BOOST_FOREACH(int frame_id, test_ids) {
		TITLE("Reconstructing frame " << frame_id);

		// Get base frame
		const KeyFrame& frame = *map.KeyFrameByIdOrDie(frame_id);
		const PosedImage& image = map.ImageByIdOrDie(frame_id);
		DPGeometry geom(image.pc(), zfloor, zceil);

		// Initialize payoffs
		MatF agree_payoffs, occl_payoffs;;
		ComputeLandmarkPayoffs(frame, geom, zfloor, zceil, agree_payoffs, occl_payoffs);
		payoffs.Resize(geom.grid_size, 0);
		payoffs.Add(10.0, agree_payoffs);
		payoffs.Add(0.1, occl_payoffs);

		// Reconstruct
		recon.Compute(image, geom, payoffs);
		recon.OutputSolution(str(filepat % frame_id % "soln"));
		recon.ReportAccuracy(gt_map.floorplan());
		recon.ReportDepthError(gt_map.floorplan());

		// Draw depth
		const MatD& soln_depths = recon.dp.ComputeDepthMap(zfloor, zceil);
		WriteMatrixImageRescaled(str(filepat % frame_id % "soln_depth"), soln_depths);

		// Draw ground truth
		MatI gt_orients;
		MatD gt_depths;
		GetTrueOrients(gt_map.floorplan(), image.pc(), gt_orients, gt_depths);
		WriteOrientationImage(str(filepat % frame_id % "gt_orients"), gt_orients);
		WriteMatrixImageRescaled(str(filepat % frame_id % "gt_depth"), gt_depths);

		// Draw payoffs
		ImageRGB<byte> payoff_canvas;
		float* r = payoffs.wall_scores[0][0];
		DrawMatrixRescaled(payoffs.wall_scores[0], payoff_canvas);
		recon.dp.DrawWireframeGridSolution(payoff_canvas);
		WriteImage(str(filepat % frame_id % "payoffs"), payoff_canvas);
		WriteMatrixImageRescaled(str(filepat % frame_id % "alt_payoffs"), payoffs.wall_scores[0]);

		// Visualize
		BrightColors bc;
		FileCanvas canvas(str(filepat % frame_id % "proj"), image.rgb);
		BOOST_FOREACH(const Measurement& msm, frame.measurements) {
			Vec3 pt = map.pts[msm.point_index];
			Vec2 im_pt = project(image.pc().WorldToIm(pt));
			if (image.contains(asIR(im_pt))) {
				Vec3 f = pt;
				Vec3 c = pt;
				f[2] = zfloor;
				c[2] = zceil;
				Vec2 im_f = project(image.pc().WorldToIm(f));
				Vec2 im_c = project(image.pc().WorldToIm(c));
				PixelRGB<byte> color = bc.Next();
				canvas.DrawDot(im_pt, 4.0, color);
				canvas.DrawDot(im_f, 2.5, color);
				canvas.DrawDot(im_c, 2.5, color);
				canvas.StrokeLine(im_f, im_c, color);
			}
		}
	}

	return 0;
}
