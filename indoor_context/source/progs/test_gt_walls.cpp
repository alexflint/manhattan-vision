#include <iostream>
#include <queue>

#include <LU.h>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"
#include "bld_helpers.h"
#include "image_utils.h"
#include "canvas.h"
#include "geom_utils.h"
#include "manhattan_ground_truth.h"
#include "line_segment.h"

#include "format_utils.tpp"
#include "counted_foreach.tpp"

using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);

	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath("lab_kitchen1"), gt_map);

	ManhattanGroundTruth gt;
	for (int i = 0; i < map.kfs.size(); i += 5) {
		Frame& frame = map.kfs[i];
		frame.LoadImage();

		gt.Compute(gt_map.floorplan(), frame.image.pc());
		TITLED("Frame " << frame.id) DREPORT(gt.num_walls(), gt.num_occlusions());

		// Draw the orientations
		ImageRGB<byte> orients;
		DrawOrientations(gt.orientations(), orients);
		FileCanvas canvas(fmt("out/frame%03d_verts.png", frame.id), orients);
		canvas.SetLineWidth(4.);
		CHECK_EQ(gt.floor_segments.size(), gt.ceil_segments.size());
		COUNTED_FOREACH(int j, const LineSeg& seg, gt.floor_segments) {
			canvas.StrokeLine(seg, gt.segment_orients[j] ? Colors::black() : Colors::white());
		}
		COUNTED_FOREACH(int j, const LineSeg& seg, gt.ceil_segments) {
			canvas.StrokeLine(seg, gt.segment_orients[j] ? Colors::black() : Colors::white());
		}
	}
	return 0;
}
