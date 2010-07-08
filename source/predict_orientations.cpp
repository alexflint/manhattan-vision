/*
 * predict_orientations.cpp
 *
 *  Created on: 1 Jun 2010
 *      Author: alexf
 */

#include <VW/Image/imageio.h>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"
#include "vars.h"
#include "floorplan_renderer.h"
#include "image_utils.h"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro INDEX";
		return 0;
	}

	// Load the truthed map
	proto::TruthedMap gt_map;
	ifstream str(argv[1], ios::binary);
	CHECK(gt_map.ParseFromIstream(&str)) << "Failed to read from " << argv[1];

	// Load the map
	Map map;
	map.LoadXml(gt_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(gt_map.ln_scene_from_slam())));

	KeyFrame& kf = *map.KeyFrameById(atoi(argv[2]));

	// Predict some orientations
	FloorplanRenderer r(gt_map);
	MatI orients;
	r.PredictOrients(*kf.pc, orients);
	WriteOrientationImage("out/predicted.png", orients);

	kf.LoadImage();
	VW::WriteImage("out/orig.png", kf.image.rgb);

	return 0;
}
