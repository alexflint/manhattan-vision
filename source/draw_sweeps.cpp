#include <fstream>

#include <LU.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "entrypoint_types.h"
#include "canvas.tpp"
#include "vars.h"
#include "map.h"
#include "map.pb.h"

#include "bld_helpers.h"
#include "guided_line_detector.h"
#include "vanishing_points.h"
#include "line_sweeper.h"

#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "range_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

Mat3 ComputeVerticalRectifier(const PosedCamera& pc) {
	Vec3 up = pc.GetRetinaVpt(2);
	if (up[1] < 0) up = -up;
	Mat3 R_up;
	R_up[0] = up ^ GetAxis<3>(2);
	R_up[1] = up;
	R_up[2] = R_up[0] ^ R_up[1];

	// T_centre projects the centre of the original image projects to
	// the centre of the new image
	Mat3 T_centre = Identity;
	T_centre.slice<0,2,2,1>() = -project(R_up*makeVector(0,0,1)).as_col();

	// Compute the warping homographies to transform the image so that
	// vertical in the world is vertical in the image.
	Mat3 H_canon = T_centre * R_up;

	// Compute scaling to keep all corners within bounds
	double scale = 1.0;
	Polygon<4> ret_perimeter = pc.retina_bounds().GetPolygon();
	for (int i = 0; i < 4; i++) {
		double canon_x = project(H_canon * ret_perimeter.verts[i])[0];
		// this works because the retina is centred at zero...
		scale = max(scale, canon_x/pc.retina_bounds().left());
		scale = max(scale, canon_x/pc.retina_bounds().right());
	}

	// Since H_canon operates on homogeneous coordinates we must not
	// apply the scaling to the third row
	DiagonalMatrix<3> M_scale(makeVector(1.0/scale, 1.0/scale, 1.0));

	return H_canon * M_scale;
}

void RectifyVertical(const PosedCamera& pc,
                     const ImageRGB<byte>& in,
                     ImageRGB<byte>& out) {
	Mat3 H = ComputeVerticalRectifier(pc);
	Mat3 Hinv = LU<>(H).get_inverse();
	out.AllocImageData(in.GetWidth(), in.GetHeight());
	out.Clear(Colors::white());

	for (int y = 0; y < in.GetHeight(); y++) {
		for (int x = 0; x < in.GetWidth(); x++) {
			ImageRef p = asIR(project(pc.RetToIm(Hinv*pc.ImToRet(makeVector(x,y,1.0)))));
			if (p.x >= 0 && p.x < in.GetWidth() &&
					p.y >= 0 && p.y < in.GetHeight()) {
				out[y][x] = in[p];
			}
		}
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3 && argc != 4) {
		DLOG<<"Usage: "<<argv[0]<<" truthed_map.pro FRAMEID [--swap_colors]";
		exit(-1);
	}

	const char* file = argv[1];
	int frame_id = atoi(argv[2]);
	bool swap_vert_labels = argc > 3;

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(file, gt_map);

	KeyFrame& kf = *map.KeyFrameByIdOrDie(frame_id);
	kf.LoadImage();

	GuidedLineDetector line_detector(kf.image);
	IsctGeomLabeller sweeps(kf.image, line_detector.detections);
	if (swap_vert_labels) {
		InterchangeLabels(sweeps.orient_map, 0, 1);
	}
				
	// Draw the orientation into an image
	ImageRGB<byte> orient_canvas;
	ImageCopy(kf.image.rgb, orient_canvas);
	sweeps.DrawOrientViz(orient_canvas);

	// Replace black pixels with white pixels
	for (int y = 0; y < orient_canvas.GetHeight(); y++) {
		PixelRGB<byte>* row = orient_canvas[y];
		for (int x = 0; x < orient_canvas.GetWidth(); x++) {
			if (row[x] == Colors::black()) row[x] = Colors::white();
		}
	}

	ImageRGB<byte> rect_orient_canvas;
	RectifyVertical(*kf.pc, orient_canvas, rect_orient_canvas);
	WriteImage(str(format("out/frame%03d_sweeps.png") % kf.id), rect_orient_canvas);

	return 0;
}
