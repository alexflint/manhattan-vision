#include "entrypoint_types.h"
#include "manhattan_dp.h"
#include "vars.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"
#include "timer.h"
#include "clipping.h"
#include "bld_helpers.h"
#include "safe_stream.h"
#include "line_sweep_features.h"
#include "canvas.h"

#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

lazyvar<int> gvBeginFrame("Tata.BeginFrame");
lazyvar<int> gvBeginCoords("Tata.BeginCoords");
lazyvar<int> gvBeginModel("Tata.BeginModel");
lazyvar<int> gvEndFrame("Tata.EndFrame");
lazyvar<int> gvStride("Tata.Stride");

void DrawAxes(Canvas& canvas, const PosedCamera& pc) {
	Vec2 image_ctr = pc.RetToIm(makeVector(0.0, 0.0));
	Vec3 scene_ctr = pc.pose_inverse() * makeVector(0,0,1);
	canvas.SetLineWidth(3.0);
	for (int j = 0; j < 3; j++) {
		Vec3 retina_ej = pc.pose() * (scene_ctr + GetAxis<3>(j));
		Vec2 image_ej = pc.RetToIm(project(retina_ej));
		canvas.StrokeLine(image_ctr, image_ej, BrightColors::Get(j));
	}
}

void ProcessFrame(Frame& frame, const KeyFrame* last_kf, string outfile,
									const Map& map, const proto::TruthedMap& gt_map) {
	DLOG << "Processing frame " << frame.id;

	// Load the image
	frame.LoadImage();

	// Draw the annotation
	if (frame.id < *gvBeginCoords) {
		if (last_kf == NULL) {
			WriteImage(outfile, frame.image.rgb);
		} else {
			FileCanvas canvas(outfile, frame.image.rgb);
			BOOST_FOREACH(const Measurement& msm, last_kf->measurements) {
				const Vec3& p = map.pts[msm.point_index];
				canvas.DrawDot(project(frame.image.pc().WorldToIm(p)), 2.0, Colors::red());
			}
		}
	} else if (frame.id < *gvBeginModel) {
		FileCanvas canvas(outfile, frame.image.rgb);
		DrawAxes(canvas, frame.image.pc());

	} else {
		MatI gt_orients;
		GetTrueOrients(gt_map.floorplan(), frame.image.pc(), gt_orients);
		DrawOrientations(gt_orients, frame.image.rgb, 0.35);
		WriteImage(outfile, frame.image.rgb);
	}

	// Unload the image
	frame.UnloadImage();

}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" SEQUENCE";
		return -1;
	}

	const string& sequence = argv[1];

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	int out_index = 0;

	const KeyFrame* last_kf = NULL;
	int next_kf = 1;  // skip the first

	Worker workers[4];

	int end = min(*gvEndFrame, map.frames.size()-1);
	if (end == -1) end = map.frames.size()-1;
	DREPORT(end);
	for (int i = *gvBeginFrame; i <= end; i+=*gvStride) {
		CHECK_INDEX(i, map.frames);

		format filepat("out/%05d.png");
		string outfile = str(filepat % out_index);
		out_index++;

		Frame& frame = map.frames[i];
		if (next_kf < map.kfs.size() && map.kfs[next_kf].image_file.compare(frame.image_file) <= 0) {
			last_kf = &map.kfs[next_kf++];
		}

		workers[i%4].Add(bind(&ProcessFrame, ref(frame), last_kf, outfile, ref(map), ref(gt_map)));
	}

	for (int i = 0; i < 4; i++) {
		workers[i].Join();
	}

	return 0;
}
