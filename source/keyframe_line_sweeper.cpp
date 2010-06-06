#include <iomanip>

#include <LU.h>

#include "common_types.h"
#include "map_widgets.h"
#include "map.h"
#include "textons.h"
#include "vars.h"
#include "viewer3d.h"
#include "line_sweeper.h"
#include "gl_utils.tpp"
#include "timer.h"

using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map and rotate to canonical frame
	Map map;
	map.Load();
	map.RotateToSceneFrame();

	COUNTED_FOREACH(int i, const KeyFrame& kf, map.kfs) {
		DREPORT(i);
		INDENTED {
			SegGeomLabeller labeller;
			TIMED("Complete")	labeller.Compute(kf.unwarped.image, kf.vpts);
			string stri = lexical_cast<string>(i);
			kf.vpts.OutputVanPointViz("out/"+stri+"_vpts.png");
			labeller.OutputOrientViz("out/"+stri+"_orients.png");
			labeller.sweeper.OutputSupportViz("out/"+stri+"_support");
		}
	}

	return 0;
}
