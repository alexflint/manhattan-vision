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
	WITHOUT_DLOG kf.RunGuidedLineDetector();
	boost::format fmt("out/frame%02d_%s.png");

	WriteImage(str(fmt % kf.id % "orig"), kf.image.rgb);

	CannyLineDetector canny(kf.image);
	canny.OutputLineViz(str(fmt % kf.id % "canny"));

	FileCanvas canny_canvas(str(fmt % kf.id % "canny"), makeVector(640,480));
	canny_canvas.DrawImage(kf.image.rgb);
	canny_canvas.SetLineWidth(3.0);
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
		canny_canvas.StrokeLine(det.seg, color);
	}

	GuidedLineDetector& guided = kf.guided_line_detector;
	FileCanvas guided_canvas(str(fmt % kf.id % "guided"), makeVector(640,480));
	guided_canvas.DrawImage(kf.image.rgb);
	guided_canvas.SetLineWidth(3.0);
	for (int i = 0; i < 3; i++) {
		BOOST_FOREACH(const LineDetection& det, guided.detections[i]) {
			guided_canvas.StrokeLine(det.seg, Colors::primary(i));
		}
	}
	//det.OutputSegmentsViz(str(fmt % kf.id % "guided"));

	/*
	WriteImage(str(fmt % kf.id % "orig"), kf.image.rgb);
	det.OutputAssociationViz(str(fmt % kf.id % "assocs"));
	det.OutputRayViz(str(fmt % kf.id % "rays"));

	WriteMatrixImageRescaled(str(fmt % kf.id % "response0"), det.responses[0]);
	WriteMatrixImageRescaled(str(fmt % kf.id % "response1"), det.responses[1]);
	WriteMatrixImageRescaled(str(fmt % kf.id % "response2"), det.responses[2]);

	double max_support = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < det.histogram[i].size(); j++) {
			if (det.histogram[i][j].support > max_support) DREPORT(i,j,det.histogram[i][j].support);
			max_support = max(max_support, det.histogram[i][j].support);
		}
	}

	for (int i = 0; i < 3; i++) {
		string stri = lexical_cast<string>(i);

		ImageRGB<byte> canvas(kf.image.sz());
		for (int y = 0; y < kf.image.ny(); y++) {
			for (int x = 0; x < kf.image.nx(); x++) {
				int bin = det.GetBin(x,y,i);
				double v = det.histogram[i][bin].support / max_support;
				canvas[y][x] = Colors::grey(sqrt(v));
			}
		}
		WriteImage(str(fmt % kf.id % (string("bins")+stri)), canvas);

		vector<int> hist;
		BOOST_FOREACH(const LineBin& bin, det.histogram[i]) {
			hist.push_back(bin.support);
		}

		ImageRGB<byte> hist_canvas;
		DrawHistogram(hist_canvas, hist, 0.75);
		WriteImage(str(fmt % kf.id % (string("hist")+stri)), hist_canvas);

		det.OutputRayViz(str(fmt % kf.id % (string("rays")+stri)), i);
		det.OutputSegmentsViz(str(fmt % kf.id % (string("segments")+stri)), i);
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
