#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include "entrypoint_types.h"
#include "viewer3d.h"
#include "widget3d.h"
#include "vars.h"
#include "read_ply.h"
#include "colored_points.h"
#include "map_widgets.h"

#include "gl_utils.tpp"

using namespace indoor_context;
using namespace std;

void Normalize(vector<pair<Vec3, PixelRGB<byte> > >& points) {
	Vec3 sum = Zeros;
	for (int i = 0; i < points.size(); i++) {
		sum += points[i].first;
	}
	Vec3 centre = sum / points.size();

	double radius = 0;
	for (int i = 0; i < points.size(); i++) {
		radius = max(radius, norm(points[i].first - centre));
	}

	double scale = 10. / radius;
	for (int i = 0; i < points.size(); i++) {
		points[i].first = scale * (points[i].first - centre);
	}
}
	

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2 && argc != 3) {
		cerr << "Usage: "<<argv[0]<<" file.ply [map.xml]\n";
		return -1;
	}

	CHECK_PRED(exists, fs::path(argv[1]));

	// Read the .ply file
	ColoredPoints points;
	ReadPly(argv[1], points.vs);
	Normalize(points.vs);

	DREPORT(points.vs.size());
	

	// Initialize the viewer
	Viewer3D viewer;
	viewer.Add(points, 'p');

	viewer.AddOwned(new GroundPlaneWidget);

	// Create the map
	scoped_ptr<Map> map;
	if (argc == 3) {
		map.reset(new Map);
		map->LoadXml(argv[2]);
		viewer.AddOwned(new MapWidget(map.get()), 'm');
	}

	// Run the view
	viewer.Run();

	return 0;
}
