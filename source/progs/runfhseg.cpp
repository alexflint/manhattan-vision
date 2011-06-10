#include <VW/Image/imagecopy.h>
#include <VW/Utils/timer.h>

#include "common_types_vw.h"
#include "fhsegmenter.h"
#include "timer.h"
#include "misc.h"
#include "image_bundle.h"

int main(int argc, char **argv) {
	InitVars(argc, argv, "common.cfg");
	if (argc != 2) {
		cerr << "Usage: runfhseg IMAGE" << endl;
		exit(-1);
	}

	ImageBundle image(argv[1]);
	FHSegmenter segmenter;
	TIMED("Segment") segmenter.Compute(image);
	DLOG << "Produced " << segmenter.num_segments  << " segments";
	segmenter.OutputSegViz("out/segmentation.png");

	return 0;
}
