#include <VW/Image/imagecopy.h>
#include <VNL/vector.tpp>

#include "vars.h"
#include "common_types_vw.h"
#include "timer.h"
#include "vanishing_points.h"
#include "image_bundle.h"

#include "image_utils.tpp"

using namespace indoor_context;

lazyvar<Vec5> gvDefaultCameraParams("Map.DefaultCameraParameters");
lazyvar<Vec2> gvDefaultImageSize("Map.DefaultImageSize");

int main(int argc, char **argv) {
	InitVars(argc, argv);

	// Load image
	ATANCamera camera(*gvDefaultCameraParams, *gvDefaultImageSize);
	CalibratedImage image(&camera, argv[1]);

	// Find vanishing points
	VanishingPoints vpts;
	TIMED("Find vanishing points") vpts.Compute(image);

	// Output vizualizations
	//WriteMatrixImageRescaled("out/magnitude_sqr.png",
	//												 vpts.line_detector.canny.magnitude_sqr);
	//WriteMatrixImageRescaled("out/edges.png",
	//												 vpts.line_detector.canny.edge_map);
	vpts.OutputLineViz("out/linesegs.png");
	vpts.OutputVanPointViz("out/vpts.png");

	return 0;
}
