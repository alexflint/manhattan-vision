#include <LU.h>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"
#include "geom_utils.h"
#include "colors.h"

#include "io_utils.tpp"

using indoor_context::ParseMultiRange;
using indoor_context::ParseMultiRange;

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc != 3) {
		cout << "Usage: "<<argv[0]<<" TRUTHED_MAP FRAME";
		return -1;
	}

	const char* path = argv[1];
	vector<int> frameIds = ParseMultiRange<int>(string(argv[2]));

	Map map;
	proto::TruthedMap tru_map;
	map.LoadWithGroundTruth(path, tru_map);

	BOOST_FOREACH(int frameId, frameIds) {
		KeyFrame* frame = map.KeyFrameByIdOrDie(frameId);
		frame->LoadImage();

		Bounds2D<> in_bounds = Bounds2D<>::FromTightSize(asToon(frame->pc->image_size()));
		Bounds2D<> out_bounds(100, 400, 50, 450);

		Mat3 m = GetVerticalRectifier(*frame->pc, out_bounds);
		Mat3 m_inv = LU<>(m).get_inverse();

		ImageRGB<byte> out(500, 500);
		out.Clear(Colors::white());
		for (int y = 0; y < out.GetHeight(); y++) {
			for (int x = 0; x < out.GetWidth(); x++) {
				if (y == out_bounds.top() || y == out_bounds.bottom() ||
						x == out_bounds.left() || x == out_bounds.right()) {
					out[y][x] = Colors::black();
				} else {
					Vec2I v = project(m_inv * makeVector(x,y,1.0));
					if (in_bounds.Contains(v)) {
						out[y][x] = frame->image.rgb[ v[1] ][ v[0] ];
					}
				}
			}
		}
		WriteImage("out/transformed.png", out);

		frame->UnloadImage();
	}



	return 0;
}
