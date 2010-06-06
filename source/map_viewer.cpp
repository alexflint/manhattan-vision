#include <iomanip>

#include <LU.h>
#include <so3.h>

#include "common_types.h"
#include "map_widgets.h"
#include "map.h"
#include "map.pb.h"
#include "textons.h"
#include "vars.h"
#include "viewer3d.h"

#include "math_utils.tpp"
#include "gl_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);

	proto::TruthedMap tru_map;
	ifstream tru_in(argv[1], ios::binary);
	CHECK(tru_map.ParseFromIstream(&tru_in)) << "Failed to read from " << argv[1];
	SO3<> scene_from_slam = SO3<>::exp(asToon(tru_map.ln_scene_from_slam()));
	DREPORT(scene_from_slam);

	if (argc > 2) {
		DLOG << "Usage: "<<argv[0]<<" [--random_rotation]";
		return 0;
	}

	// Load the map
	MapViz mapviz;
	if (argc == 2) {
		mapviz.map.RotateToSceneFrame(scene_from_slam);
	} else {
		mapviz.map.RotateToSceneFrame(RandomVector<3>());
	}
	//mapviz.map.RunGuidedLineDetectors();

	// Enter the event loop
	mapviz.Run();

	return 0;
}
