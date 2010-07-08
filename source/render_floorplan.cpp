/*
 * test_render.cpp
 *
 *  Created on: 6 Jul 2010
 *      Author: alexf
 */

#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "floorplan_renderer.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"

#include "image_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG	<< "Usage: " << argv[0] << " truthed_map.pro INDEX";
		return 0;
	}

	int kf_index = atoi(argv[2]);

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(argv[1], ios::binary);
	CHECK(tru_map.ParseFromIstream(&s)) << "Failed to read from " << argv[1];

	// Load the map
	Map map;
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	// Pull out the frame
	const KeyFrame* kf = map.KeyFrameByIdOrDie(kf_index);

	FloorPlanRenderer re;

	ImageRGB<byte> canvas;
	re.Render(tru_map.floorplan(), *kf->pc, canvas);
	WriteImage("out/canvas.png", canvas);

	MatI orients;
	re.RenderOrients(tru_map.floorplan(), *kf->pc, orients);
	WriteOrientationImage("out/orients.png", orients);

	fs::copy_file(kf->image_file, string("out/frame.png"));

	/*
	// Make the camera matrix
	Mat3 linear_intr;
	LinearCamera::Linearize(kf->pc->camera, linear_intr);
	toon::Matrix<3,4> cam = linear_intr * as_matrix(kf->pc->pose);

	// Set up the renderer
	SimpleRenderer re(cam, asToon(kf->pc->im_size()));
	re.Clear(2);

	// Do the rendering
	const proto::FloorPlan& fp = tru_map.floorplan();
	for (int i = 0; i < fp.vertices_size(); i++) {
		Vec2 u = asToon(fp.vertices(i));
		Vec2 v = asToon(fp.vertices((i+1)%fp.vertices_size()));
		if (isnan(u) || isnan(v)) continue;

		Vec3 p = concat(u, fp.zceil());
		Vec3 q = concat(v, fp.zceil());
		Vec3 r = concat(v, fp.zfloor());
		Vec3 s = concat(u, fp.zfloor());

		int label = abs(u[0]-v[0]) > abs(u[1]-v[1]) ? 0 : 1;
		re.Render(p, q, r, label);
		re.Render(p, r, s, label);

		TITLE(i);
		DREPORT(p,q,r,s);
		DREPORT(cam*unproject(p), cam*unproject(q), cam*unproject(r), cam*unproject(s));

		// Visualize
		WriteOrientationImage(str(format("out/orients_pre%d.png")%i), re.framebuffer());
	}

	// Visualize
	WriteOrientationImage("out/orients.png", re.framebuffer());*/

	return 0;
}
