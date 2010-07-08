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

#include "math_utils.tpp"
#include "gl_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);

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
	Map map;
	map.LoadXml(tru_map.spec_file());
	if (argc == 2) {
		map.RotateToSceneFrame(scene_from_slam);
	} else {
		map.RotateToSceneFrame(RandomVector<3>());
	}

	// Enter the event loop
	Viewer3D v;
	//v.AddOwned(new MapWidget(&map));
	v.Run();

	return 0;
}
