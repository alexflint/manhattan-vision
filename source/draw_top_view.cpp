/*
 * draw_map.cpp
 *
 *  Created on: 26 May 2010
 *      Author: alexf
 */

#include <TooN/sl.h>

#include "common_types_entry.h"
#include "map.h"
#include "map.pb.h"
#include "canvas.h"
#include "vars.h"
#include "colors.h"
#include "geom_utils.h"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro";
		return 0;
	}

	// Load the truthed map
	proto::TruthedMap gt_map;
	ifstream str(argv[1], ios::binary);
	CHECK(gt_map.ParseFromIstream(&str)) << "Failed to read from " << argv[1];

	// Load the map
	Map map;
	map.LoadXml(gt_map.spec_file());
	//map.RotateToSceneFrame(SO3<>::exp(RandomVector<3>()));
	map.RotateToSceneFrame(SO3<>::exp(asToon(gt_map.ln_scene_from_slam())));

	// Draw the floorplan from the top
	Vec2 sz = makeVector(1000,1000);
	FileCanvas canvas("out/top_view.png", sz);

	Matrix<2,3> zproj = Zeros;
	zproj.slice<0,0,2,2>() = Identity;

	// Compute bounds
	vector<Vec3> map_items;
	copy_all_into(map.pts, map_items);
	BOOST_FOREACH(const KeyFrame& kf, map.kfs) {
		map_items.push_back(kf.pc->world_centre());
	}
	pair<Vec3,Vec3> bounds = ComputeBounds<3>(map_items);
	Vec2 u = zproj*bounds.first;
	Vec2 v = zproj*bounds.second;
	double s = min(sz[0]/(v[0]-u[0]), sz[1]/(v[1]-u[1]));
	Vec2 t = -u*s;

	// Draw the points
	BOOST_FOREACH(const Vec3& pt, map.pts) {
		canvas.DrawDot(s*zproj*pt+t, Colors::red());
	}

	// Draw the camera trajectory
	for (int i = 0; i < map.kfs.size(); i++) {
		Vec2 camc = zproj * map.kfs[i].pc->world_centre();
		Vec3 fwd = map.kfs[i].pc->invpose.get_rotation() * makeVector(0,0,1);
		Vec2 side = zproj * (fwd ^ makeVector(0,0,1));
		canvas.StrokeLine(s*(camc-side*0.3)+t, s*(camc+side*0.3)+t, Colors::black());
		if (i+1 < map.kfs.size()) {
			Vec2 next = zproj * map.kfs[i+1].pc->world_centre();
			canvas.StrokeLine(s*camc+t, s*next+t, Colors::black());
		}
	}

	const proto::FloorPlan& fp = gt_map.floorplan();
	for (int i = 0; i < fp.vertices_size()-1; i++) {
		Vec2 a = asToon(fp.vertices(i));
		Vec2 b = asToon(fp.vertices(i+1));
		canvas.StrokeLine(s*a+t, s*b+t, Colors::blue());
	}

	canvas.Write();

	return 0;
}


