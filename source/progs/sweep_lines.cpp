#include "entrypoint_types.h"
#include "line_sweeper.h"
#include "guided_line_detector.h"
#include "map.h"
#include "map_io.h"
#include "map.pb.h"

#include "bld_helpers.h"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: sweep_lines FRAME_ID";
		exit(-1);
	}

	string sequence = "lab_kitchen1";
	int frame_id = atoi(argv[1]);

	Map map;
	proto::TruthedMap gt_map;
	string path = GetMapPath(sequence);
	LoadXmlMapWithGroundTruth(path, map, gt_map);

	Frame* f = map.GetFrameByIdOrDie(frame_id);
	DLOG << "Loading image...";
	f->LoadImage();
	DLOG << "Loaded image.";

	GuidedLineDetector line_detector;
	line_detector.Compute(f->image);

	IsctGeomLabeller sweeper;
	sweeper.Compute(f->image, line_detector.detections);

	sweeper.OutputOrientViz("out/orients.png");

	return 0;
}

