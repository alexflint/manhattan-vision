// Compare joint versus single image vpt recovery, for CVPR 2010 paper

#include <iomanip>

#include <LU.h>

#include "common_types.h"
#include "map_widgets.h"
#include "map.h"
#include "textons.h"
#include "vars.h"
#include "viewer3d.h"
#include "gl_utils.tpp"
#include "map.pb.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
//#include "numeric_utils.tpp"

using namespace indoor_context;
using namespace toon;


void DrawVptViz(ImageRGB<byte>& canvas,
								const PosedImage& pim,
								const vector<LineDetector>& detections) const {
	// Copy the original image into the vizualization
	Vector<2> offset = makeVector(*gvVizPadding, *gvVizPadding);
	canvas.AllocImageData(orig.nx()+offset[0]*2, orig.ny()+offset[1]*2);
	canvas.Clear(Colors::white());
	CopyImageInto(orig.rgb, offset[1], offset[0], canvas);

	// Compute vanishing point locations in the image
	BOOST_FOREACH (const LineDetection& det, detections) {
		// Draw the extension
		if (det.axis != -1) {
			Vector<3> a = unit(det.seg.start);
			Vector<3> b = unit(det.seg.end);
			const Vector<3>& vpt = image_vpts[det.axis];
			const Vector<3>& endpt = a*vpt > b*vpt ? a : b;
			DrawLineClipped(canvas,
											project(endpt) + offset,
											project(vpt) + offset,
											Colors::primary(det.axis),
											*gvVanLineAlpha);
		}

		// Draw the line segment
		DrawLine(canvas,
						 project(det.seg.start) + offset,
						 project(det.seg.end) + offset,
						 det.axis == -1 ? kSpuriousColor : Colors::primary(det.axis),
						 *gvVanLineAlpha);
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the truthed map
	string tru_file = GV3::get<string>("VptComparison.TruthedMap");
	proto::TruthedMap tru_map;
	ifstream s(tru_file.c_str(), ios::binary);
	CHECK(tru_map.ParseFromIstream(&s)) << "Failed to read from " << tru_file;

	// Read the KF ids
	vector<int> kf_ids;
	ParseMultiRange(GV3::get<string>("VptComparison.OutputLineKFs"), kf_ids);

	// Load the map
	Map map;
	map.kf_ids_to_load = kf_ids;
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	BOOST_FOREACH (KeyFrame& kf, map.kfs) {
		PosedImage pim(kf.pc);
		ImageCopy(kf.image.rgb, pim);

		// Output jointly estimated vanishing points
		ImageRGB<byte> joint_canvas;
		DrawVptViz(joint_canvas, pim, kf.line_detector.detections);
		WriteImage("out/joint_"+PaddedInt(kf.id, 8)+"_lines.png", joint_canvas);

		ManhattanEstimator indiv_est;

		// Now do single image
		// At this point the line segments have already been conditioned
		kf.manhattan_est.Compute(kf.line_detector.detections);
		// Undo the conditioning for vizualization
		for (int i = 0; i < 3; i++) {
			manhattan_est.vpts[i] = kf.vpt_homog_inv * manhattan_est.vpts[i];
		}

		// Output individually estimated vanishing points
		ImageRGB<byte> indiv_canvas;
		DrawVptViz(joint_canvas, pim, kf.line_detector.detections);
		WriteImage("out/indiv_"+PaddedInt(kf.id, 8)+"_lines.png", indiv_canvas);
	}

	return 0;
}
