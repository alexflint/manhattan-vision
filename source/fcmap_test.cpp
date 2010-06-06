#include "floor_ceil_map.h"
#include "common_types.h"
#include "vars.h"
#include "viewer3d.h"
#include "map.h"
#include "map_widgets.h"
#include "map.pb.h"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

lazyvar<int> gvFrameId("ManhattanDP.FrameId");

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro";
		return 0;
	}

	Viewer3D viewer;

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(argv[1], ios::binary);
	CHECK(tru_map.ParseFromIstream(&s));

	// Find the truthed frame
	const proto::TruthedFrame* tru_frame = NULL;
	BOOST_FOREACH(const proto::TruthedFrame& cur, tru_map.frame()) {
		if (cur.id() == *gvFrameId) {
			tru_frame = &cur;
		}
	}
	CHECK_NOT_NULL(tru_frame);

	// Load the map
	Map map;
	map.kf_ids_to_load.clear();
	map.kf_ids_to_load.push_back(*gvFrameId);
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	// Find the frame
	const KeyFrame& kf = *map.KeyFrameById(tru_frame->id());
	CHECK_NOT_NULL(&kf);

	// Construct a pair of floor/ceiling points
	Vector<3> ref_pt = kf.pc->invpose * (GetAxis<3>(2) * 10.0);
	Vector<3> floor_pt = ref_pt, ceil_pt = ref_pt;
	floor_pt[2] = tru_map.floorplan().zfloor();
	ceil_pt[2] = tru_map.floorplan().zceil();

	swap(floor_pt, ceil_pt);

	// Project to the image
	Vector<3> ret_floor_pt = kf.pc->WorldToRet(floor_pt);
	Vector<3> ret_ceil_pt = kf.pc->WorldToRet(ceil_pt);

	FloorCeilMap fcmap;
	fcmap.Compute(ret_floor_pt, ret_ceil_pt, *kf.pc);

	// Add things to the viewer
	MapWidget mapWidget(&map);
	KeyFrameWidget& kfWidget = *mapWidget.kf_widgets[0];
	viewer.Add(mapWidget);
	viewer.AddOwned(new LineWidget(floor_pt, ceil_pt, 2.0, Colors::aqua()));
	kfWidget.AddLineInRetina(ret_floor_pt, ret_ceil_pt, 2.0, Colors::aqua());

	kfWidget.AddLineInRetina(ret_floor_pt, fcmap.Transfer(ret_floor_pt), 2.0, Colors::aqua());


	// Add lines to the viewer
	for (int y = 0; y <= 600; y+=100) {
		for (int x = 0; x <= 400; x+=100) {
			Vector<3> ret_a = kf.pc->ImToRet(makeVector(x,y,1.0));
			Vector<3> ret_b = fcmap.Transfer(ret_a);
			kfWidget.AddLineInRetina(ret_a, ret_b, 1.0, Colors::white());
		}
	}
	
	viewer.Run();

	return 0;
}

