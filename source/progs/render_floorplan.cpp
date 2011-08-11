/*
 * test_render.cpp
 *
 *  Created on: 6 Jul 2010
 *      Author: alexf
 */

#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "floorplan_renderer.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"
#include "bld_helpers.h"

#include "image_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG	<< "Usage: " << argv[0] << " truthed_map.pro INDEX";
		return 0;
	}

	int kf_index = atoi(argv[2]);

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(argv[1], ios::binary);
	CHECK(tru_map.ParseFromIstream(&s)) << "Failed to read from " << argv[1];

	// Load the map
	Map map;
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	// Pull out the frame
	const KeyFrame* kf = map.KeyFrameByIdOrDie(kf_index);
	FloorPlanRenderer re;

	ImageRGB<byte> canvas;
	re.Render(tru_map.floorplan(), *kf->pc, canvas);
	WriteImage("out/canvas.png", canvas);

	MatI orients;
	re.RenderOrients(tru_map.floorplan(), *kf->pc, orients);
	WriteOrientationImage("out/orients.png", orients);

	MatI gt_orients;
	GetTrueOrients(tru_map.floorplan(), *kf->pc, gt_orients);
	WriteOrientationImage("out/gt_orients.png", gt_orients);

	if (!fs::exists("out/frame.png")) {
		fs::copy_file(kf->image_file, "out/frame.png");
	}

	return 0;
}
