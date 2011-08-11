#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"

#include "camera.h"
#include "bld_helpers.h"
#include "manhattan_dp.h"

#include "io_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro";
		return -1;
	}

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(argv[1], gt_map);

	// Process each frame
	BOOST_FOREACH(const KeyFrame& kf, map.kfs) {
		Mat3 fToC = GetFloorCeilHomology(*kf.pc, gt_map.floorplan());
		DPGeometry geom(kf.pc, fToC);
		TITLED(kf.id) DREPORT(geom.grid_floorToCeil[1][1], geom.grid_ceilToFloor[1][1]);
	}

	return 0;
}
