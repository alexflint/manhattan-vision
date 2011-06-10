#include "entrypoint_types.h"
#include "line_sweeper.h"
#include "map.h"
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
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	KeyFrame* kf = map.KeyFrameByIdOrDie(frame_id);
	kf->LoadImage();

	GuidedLineDetector line_detector;
	line_detector.Compute(kf->image);

	IsctGeomLabeller sweeper;
	sweeper.Compute(kf->image, line_detector.detections);

	sweeper.OutputOrientViz("orients.png");

	return 0;
}

