#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "floorplan_renderer.h"
#include "map.pb.h"
#include "map.h"
#include "map_io.h"
#include "camera.h"
#include "bld_helpers.h"

#include "image_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG	<< "Usage: " << argv[0] << " SEQUENCE OUTPUT.ply";
		return 0;
	}

	ofstream out(argv[2]);

	// Load the truthed map
	Map map;
	proto::TruthedMap gt_map;
	LoadXmlMapWithGroundTruth(GetMapPath(argv[1]), map, gt_map);

	const proto::FloorPlan& floorplan = gt_map.floorplan();

	vector<Vec3> fp_verts;
	for (int i = 0; i < floorplan.vertices_size(); i++) {
		Vec2 v = asToon(floorplan.vertices(i));
		if (!isnan(v[0]) && !isnan(v[1])) {
			fp_verts.push_back(makeVector(v[0], v[1], floorplan.zfloor()));
			fp_verts.push_back(makeVector(v[0], v[1], floorplan.zceil()));
		}
	}
			
	int nverts = map.points.size() + fp_verts.size();
	int nfaces = fp_verts.size()/2 - 1;

	// Write header
	out << "ply\n"
			<< "format ascii 1.0\n"
			<< "comment made by Greg Turk\n"
			<< "comment this file is a cube\n"
			<< "element vertex " << nverts << "\n"
			<< "property float x\n"
			<< "property float y\n"
			<< "property float z\n"
			<< "element face " << nfaces << "\n"
			<< "property list uchar int vertex_index\n"
			<< "end_header\n";

	// Write vertices
	BOOST_FOREACH(const Vec3 v, fp_verts) {
		out << v[0] << " " << v[1] << " " << v[2] << '\n';
	}
	BOOST_FOREACH(const Vec3 v, map.points) {
		out << v[0] << " " << v[1] << " " << v[2] << '\n';
	}

	// Write faces
	for (int i = 0; i+3 < fp_verts.size(); i += 2) {
		out << "4 " << i << " " << (i+1) << " " << (i+3) << " " << (i+2) << endl;
	}

	return 0;
}
