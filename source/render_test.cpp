/*
 * test_render.cpp
 *
 *  Created on: 6 Jul 2010
 *      Author: alexf
 */

#include "entrypoint_types.h"
#include "simple_renderer.h"

#include "image_utils.tpp"
#include "vector_utils.tpp"

int main(int argc, char **argv) {
	// Set up the polygon
	vector<Vec3> verts;
	verts.push_back(makeVector(100, 400, 1));
	verts.push_back(makeVector(400, 100, 1));
	verts.push_back(makeVector(400, 400, 1.0));

	Vec2I sz = makeVector(640, 480);

	// Camera matrices
	Mat3 intr = Identity;  // intrinsics
	toon::Matrix<3,4> extr = Zeros;
	extr.slice<0,0,3,3>() = Identity;
	toon::Matrix<3,4> cam = intr*extr;  // full camera matrix
	DREPORT(intr, extr, cam);

	// Do the rendering
	SimpleRenderer r(cam, sz);

	for (int z = -10; z <= 10; z++) {
		verts[2][2] = z/10.0;
		r.Render(verts[0], verts[1], verts[2], z+11);
	}

	// Visualize
	WriteMatrixImageRescaled("frame.png", r.framebuffer());
	WriteMatrixImageRescaled("depth.png", r.depthbuffer());

	return 0;
}
