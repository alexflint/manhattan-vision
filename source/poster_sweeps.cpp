#include <fstream>

#include "common_types_entry.h"
#include "canvas.tpp"
#include "vars.h"
#include "map.h"
#include "map.pb.h"

#include "line_detector.h"
#include "vanishing_points.h"

#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "range_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

void Process(KeyFrame& kf) {
	DREPORT(kf.id);
	kf.LoadImage();
	boost::format fmt("out/frame%02d_%s.png");

	WITHOUT_DLOG kf.RunGuidedLineDetector();
	PosedImage pim(*kf.pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	IsctGeomLabeller sweeper(pim, kf.guided_line_detector.detections);
	sweeper.OutputOrientViz(str(fmt % kf.id % "sweeps"));

	/*
	FileCanvas sweep_canvas(str(fmt % kf.id % "sweep"), makeVector(640,480));
	guided_canvas.DrawImage(kf.image.rgb);
	guided_canvas.SetLineWidth(3.0);
	for (int i = 0; i < 3; i++) {
		BOOST_FOREACH(const LineDetection& det, guided.detections[i]) {
			guided_canvas.StrokeLine(det.seg, Colors::primary(i));
		}
		}*/

	kf.UnloadImage();
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2 && argc != 3) {
		DLOG<<"Usage: "<<argv[0]<<" truthed_map.pro [INDEX]";
		exit(-1);
	}

	// Load the map
	proto::TruthedMap gt_map;
	ifstream map_in(argv[1], ios::binary);
	CHECK(gt_map.ParseFromIstream(&map_in)) << "Failed to read from " << argv[1];

	Map map;
	map.LoadXml(gt_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(gt_map.ln_scene_from_slam())));

	// Process the relevant frames
	if (argc == 3) {
		int index = atoi(argv[2]);
		CHECK_INDEX(index, map.kfs);
		Process(map.kfs[index]);
	} else {
		BOOST_FOREACH(KeyFrame& kf, map.kfs) {
			Process(kf);
		}
	}

	return 0;
}
