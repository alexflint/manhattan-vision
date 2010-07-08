#include <fstream>

#include <LU.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

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
	const Bounds2D<>& ret_bounds = pc.camera.ret_bounds();
	Polygon<4> ret_perimeter = ret_bounds.GetPolygon();
	for (int i = 0; i < 4; i++) {
		double canon_x = project(H_canon * ret_perimeter.verts[i])[0];
		// this works because the retina is centred at zero...
		scale = max(scale, canon_x/ret_bounds.left);
		scale = max(scale, canon_x/ret_bounds.right);
	}

	// Since H_canon operates on homogeneous coordinates we must not
	// apply the scaling to the third row
	DiagonalMatrix<3> M_scale(makeVector(1.0/scale, 1.0/scale, 1.0));

	return H_canon * M_scale;
}

void RectifyVertical(const PosedCamera& pc,
                     const ImageBundle& in,
                     ImageRGB<byte>& out) {
	Mat3 H = ComputeVerticalRectifier(pc);
	Mat3 Hinv = LU<>(H).get_inverse();
	out.AllocImageData(in.nx(), in.ny());
	out.Clear(Colors::white());

	for (int y = 0; y < in.ny(); y++) {
		for (int x = 0; x < in.nx(); x++) {
			ImageRef p = asIR(project(pc.RetToIm(Hinv*pc.ImToRet(makeVector(x,y,1.0)))));
			if (in.contains(p)) {
				out[y][x] = in.rgb[p];
			}
		}
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG<<"Usage: "<<argv[0]<<" truthed_map.proj";
		exit(-1);
	}

	// Load the map
	proto::TruthedMap gt_map;
	ifstream map_in(argv[1], ios::binary);
	CHECK(gt_map.ParseFromIstream(&map_in)) << "Failed to read from " << argv[1];

	Map map;
	map.LoadXml(gt_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(gt_map.ln_scene_from_slam())));

	BOOST_FOREACH(const proto::TruthedFrame& f, gt_map.frame()) {
		KeyFrame& kf = map.kfs[f.id()];
		fs::copy_file(kf.image_file,
		              str(format("out/frame%03d_orig.png") % kf.id));
		fs::copy_file(f.orient_map_file(),
		              str(format("out/frame%03d_model.png") % kf.id));

		kf.LoadImage();

		ImageRGB<byte> rect_img;
		RectifyVertical(*kf.pc, kf.image, rect_img);
		WriteImage(str(format("out/frame%03d_rectimg.png") % kf.id), rect_img);

		ImageBundle model_img(f.orient_map_file());
		ResetAlpha(model_img.rgb);
		ImageRGB<byte> rect_model;
		RectifyVertical(*kf.pc, model_img, rect_model);
		WriteImage(str(format("out/frame%03d_rectmodel.png") % kf.id), rect_model);

		kf.UnloadImage();
	}

	return 0;
}
