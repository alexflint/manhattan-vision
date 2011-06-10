#include <VW/Image/imagecopy.h>
#include <VNL/vector.tpp>

#include "common_types_vw.h"
#include "timer.h"
#include "vanishing_points.h"
#include "image_bundle.h"

#include "image_utils.tpp"

using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	ImageBundle image(argv[1]);

	// Find vanishing points
	VanishingPoints vpts;
	TIMED("Find vanishing points") vpts.Compute(image);

	// Output vizualizations
	WriteMatrixImageRescaled("out/magnitude_sqr.png",
													 vpts.lines.canny.magnitude_sqr);
	WriteMatrixImageRescaled("out/edges.png",
													 vpts.lines.canny.edge_map);
	vpts.OutputLineViz("out/linesegs.png");
	vpts.OutputVanPointViz("out/vpts.png");

	return 0;
}
