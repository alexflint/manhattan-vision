#include <cmath>

#include <iostream>
#include <sstream>

#include <gvars3/GUI.h>

#include <so3.h>
#include <LU.h>

#include "boost/bind.hpp"

#include "misc.h"
#include "common_types.h"
#include "vanishing_points.h"
#include "timer.h"
#include "image_bundle.h"
#include "unwarped_image.h"
#include "map.h"
#include "vars.h"
#include "map_widgets.h"
#include "gl_utils.tpp"

#include "image_utils.tpp"
//#include "numeric_utils.tpp"
#include "io_utils.tpp"

using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	VizTool3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map
	MapViz mapviz;

	// Load the rotation
	toon::Vector<3> lnR = GV3::get<toon::Vector<3> >("Map.LnSceneRotation");
	SO3<> R_s2w = SO3<>::exp(lnR);
	//SO3<> R_w2s = R_s2w.inverse();

	DREPORT(R_s2w);
	DREPORT(R_s2w.ln());
	mapviz.viewer.Add(bind(&MapViz::RenderAxes, ref(mapviz), R_s2w));

	/*
	// Find the bounds
	toon::Vector<3> mincorner = mapviz.map.pts[0];
	toon::Vector<3> maxcorner = mapviz.map.pts[0];
	BOOST_FOREACH(const Vector<3>& vworld, mapviz.map.pts) {
		toon::Vector<3> vscene = R_w2s * vworld;
		for (int i = 0; i < 3; i++) {
			maxcorner[i] = max(maxcorner[i], vscene[i]);
			mincorner[i] = min(mincorner[i], vscene[i]);
		}
		//scenepts.push_back(R_w2s * vscene);
	}

	DREPORT(R_s2w.ln());

	//mapviz.pts_widget->hide();
	//mapviz.viewer.Add(bind(&DrawCoordSystem, R_s2w, 10));
	mapviz.viewer.Add(bind(&DrawV, R_s2w));
	//mapviz.viewer.Add(bind(&DrawCoordSystem, R_ident, 10));
	//mapviz.viewer.Add(bind(&DrawPoints, ref(scenepts)));

	*/

	mapviz.Run();


	return 0;
}
