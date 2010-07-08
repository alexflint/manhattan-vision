#include <fstream>

#include "entrypoint_types.h"
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
	boost::format fmt("out/frame%02d_%s.%s");

	Vec2I sz = asToon(kf.image.sz());
	Vec2I offs = sz/2;

	FileCanvas vpt_canvas(str(fmt % kf.id % "vpts" % "png"), sz+offs*2);
	vpt_canvas.Translate(offs);
	vpt_canvas.DrawImage(kf.image.rgb, 1);

	CannyLineDetector canny(kf.image);
	BOOST_FOREACH(const LineDetection& det, canny.detections) {
		double best_logp = -10000;  // copied from common.cfg
		int label = -1;
		for (int i = 0; i < 3; i++) {
			double logp = ManhattanFrameEstimator::GetLogLik(kf.pc->GetImageVpt(i), det.eqn);
			if (logp > best_logp) {
				label = i;
				best_logp = logp;
			}
		}
		PixelRGB<byte> color = label == -1 ? Colors::grey() : Colors::primary(label);
		vpt_canvas.SetLineWidth(2.0);
		vpt_canvas.StrokeLine(det.seg, color);

		if (label != -1) {
			Vec3 vpt = kf.pc->GetImageVpt(label);
			vpt_canvas.StrokeLine(project(det.seg.start), project(vpt), Colors::alpha(0.2, color));
		}

	}

	vpt_canvas.Write();
		
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
