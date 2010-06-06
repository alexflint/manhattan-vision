#include <VW/Image/imagecopy.tpp>

#include "fhsegmenter.h"
#include "vanishing_points.h"
#include "common_types_vw.h"
#include "misc.h"
#include "range_utils.tpp"
#include "timer.h"
#include "geom_labeller.h"

int main(int argc, char **argv) {
	InitVars(argc, argv, "common.cfg");
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" INPUT\n";
	}

	ImageBundle image(argv[1]);
	GeomLabeller labeller(image);

	DLOG << "Generating vizualizations..." << endl;
	labeller.OutputLabelViz("out/labels.png");
	labeller.OutputAdjacencyViz("out/adj");
	labeller.vanpts.OutputLineViz("out/linesegs.png");
	labeller.vanpts.OutputVanPointViz("out/vpts.png");
	labeller.segmenter.OutputSegViz("out/segmentation.png");	

	return 0;
}
