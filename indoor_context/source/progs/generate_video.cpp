#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"
#include "joint_payoffs.h"
#include "manhattan_dp.h"
#include "map.h"
#include "floorplan_renderer.h"
#include "canvas.h"
#include "manhattan_ground_truth.h"
#include "safe_stream.h"
#include "timer.h"
#include "bld_helpers.h"
#include "progress_reporter.h"

#include "counted_foreach.tpp"
#include "format_utils.tpp"
#include "io_utils.tpp"

#include "model.pb.h"

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc != 4) {
		DLOG << "Usage: estimate_floorplan SEQUENCE MODEL VIDEONAME";
		exit(-1);
	}

	// Read parameters
	string sequence = argv[1];
	string modelfile = argv[2];
	string name = argv[3];

	// Setup output dir
	fs::path video_dir = fs::initial_path() / "videos" / name;
	if (!fs::exists(video_dir)) {
		fs::create_directory(video_dir);
	}
	CHECK_PRED1(fs::is_directory, video_dir);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();
	DREPORT(zfloor, zceil);

	// Load the model
	proto::Model model;
	ReadProto(modelfile, model);

	// Set up the file pattern
	format filepat("%s/frame%06d_%s.png");
	filepat.bind_arg(1, video_dir.string());

	// Process each frame
	FloorPlanRenderer renderer;
	ProgressReporter pro(map.frames.size());
	COUNTED_FOREACH(int index, Frame& frame, map.frames) {
		pro.Increment();
		frame.LoadImage();
		renderer.Configure(frame.image.pc());
		renderer.Render(model);
		DrawOrientations(renderer.orientations(), frame.image.rgb, 0.35);
		WriteImage(str(filepat % index % "orients"), frame.image.rgb);
		frame.UnloadImage();
	}
	
	return 0;
}
