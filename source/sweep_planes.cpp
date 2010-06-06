#include <VW/Image/imagecopy.tpp>

#include "common_types_vw.h"
#include "line_sweeper.h"
#include "vanishing_points.h"
#include "misc.h"
#include "image_bundle.h"
#include "vars.h"
#include "timer.h"

using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" IMAGE";
		return -1;
	}

	ImageBundle image(argv[1]);

	VanishingPoints vpts;
	TIMED("Detect vanishing points") vpts.Compute(image);

	IsctGeomLabeller labeller;
	TIMED("Complete")	labeller.Compute(image, vpts);
	//labeller.segmenter.OutputSegViz("out/segmentation.png");
	vpts.OutputVanPointViz("out/vpts.png");
	labeller.OutputOrientViz("out/orients.png");
	labeller.sweeper.OutputSupportViz("out/support");
	//labeller.OutputConfViz("out/confs.png");
	return 0;
}
