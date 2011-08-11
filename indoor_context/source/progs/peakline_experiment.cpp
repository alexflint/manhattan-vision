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
#include "guided_line_detector.h"

#include "image_utils.tpp"
#include "io_utils.tpp"

using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map
	Map map;
	map.Load();
	map.RotateToSceneFrame();

	//Vector<3> lnR = makeVector(1.1531, 1.26237, -1.24435);  // orig for lab_kitchen1
	//map.scene_to_slam = SO3<>::exp(lnR);
	//map.Rotate(map.scene_to_slam);

	//vector<int> outp_ids;
	//ParseMultiRange(GV3::get<string>("PeakLineExperiment.InputKFs"), outp_ids);
	BOOST_FOREACH (KeyFrame& kf, map.kfs) {
		//if (find(outp_ids.begin(), outp_ids.end(), kf.id) != outp_ids.end()) {
		DLOG << "Processing keyframe " << kf.id;
		LineDetector lines(kf.image);
		lines.OutputLineViz("out/lines_"+PaddedInt(kf.id, 3)+".png");

		PeakLines plines;
		plines.Compute(kf.image, kf.CfW, map);
		plines.OutputRayViz("out/peak_rays_"+PaddedInt(kf.id, 3)+".png");
		plines.OutputSegmentsViz("out/peak_segs_"+PaddedInt(kf.id, 3)+".png");
		//}
	}

	return 0;
}
