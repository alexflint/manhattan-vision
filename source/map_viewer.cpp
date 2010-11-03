#include <iomanip>

#include <LU.h>
#include <so3.h>

#include "entrypoint_types.h"
#include "map_widgets.h"
#include "map.h"
#include "map.pb.h"
#include "textons.h"
#include "vars.h"

#include "widget3d.h"
#include "viewer3d.h"

//#include "numeric_utils.tpp"
#include "gl_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc < 2) {
		DLOG << "Usage: "<<argv[0]<<" SEQUENCE";
		return 0;
	}

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(argv[1], gt_map);

	// Enter the event loop
	Viewer3D v;
	v.AddOwned(new MapWidget(&map));
	v.Run();

	return 0;
}
