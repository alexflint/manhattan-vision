#include <iostream>
#include <queue>

#include <LU.h>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"
#include "bld_helpers.h"
#include "image_utils.h"
#include "canvas.h"
#include "geom_utils.h"
#include "clipping.h"

using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);

	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath("lab_kitchen1"), gt_map);

	for (int i = 0; i < map.kfs.size(); i += 5) {
		KeyFrame& kf = map.kfs[i];
		kf.LoadImage();

		// Count the walls
		int num_walls, num_occlusions;
		CountVisibleWalls(gt_map.floorplan(), kf.image.pc(), num_walls, num_occlusions);
		TITLED(kf.id) DREPORT(num_walls, num_occlusions);

		// Draw the orientations
		MatI orients;
		GetTrueOrients(gt_map.floorplan(), kf.image.pc(), orients);
		WriteOrientationImage(str(format("out/frame%02d___%d_walls___%d_occl.png")
															% kf.id % num_walls % num_occlusions), orients);
	}
	return 0;
}
