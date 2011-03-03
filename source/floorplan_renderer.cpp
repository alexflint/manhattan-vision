/*
 * floorplan_renderer.cpp
 *
 *  Created on: 1 Jun 2010
 *      Author: alexf
 */

#include "floorplan_renderer.h"

#include "common_types.h"
#include "map.pb.h"
#include "camera.h"
#include "simple_renderer.h"

#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "io_utils.tpp"

namespace indoor_context {
using namespace toon;

FloorPlanRenderer::FloorPlanRenderer() {
}

void FloorPlanRenderer::Render(const proto::FloorPlan& fp,
															 const Matrix<3,4>& cam,
															 const Vec2I& viewport) {
	// Clear vectors
	walls_.clear();
	wall_labels_.clear();

	// Configure the renderer
	renderer_.Configure(cam, viewport);
	renderer_.Clear(-1);  // no pixels should still be at -1 after rendering

	// Render the floor and ceiling planes. Do _not_ do this by using
	// Clear() because then the depth map will be incomplete.
	renderer_.RenderInfinitePlane(fp.zfloor(), kVerticalAxis);
	renderer_.RenderInfinitePlane(fp.zceil(), kVerticalAxis);

	Polygon<4> wall;
	int nv = fp.vertices_size();
	for (int i = 0; i < nv; i++) {
		Vec2 u = asToon(fp.vertices(i));
		Vec2 v = asToon(fp.vertices((i+1)%nv));
		if (isnan(u) || isnan(v)) continue;

		// Construct the quad
		wall.verts[0] = concat(u, fp.zceil());
		wall.verts[1] = concat(v, fp.zceil());
		wall.verts[2] = concat(v, fp.zfloor());
		wall.verts[3] = concat(u, fp.zfloor());

		// Label surfaces by their normal direction
		int label = abs(u[0]-v[0]) > abs(u[1]-v[1]) ? 1 : 0;
		renderer_.Render(wall.verts[0], wall.verts[1], wall.verts[2], label);
		renderer_.Render(wall.verts[0], wall.verts[2], wall.verts[3], label);

		// Add to list
		walls_.push_back(wall);
		wall_labels_.push_back(label);
 	}
}

void FloorPlanRenderer::Render(const proto::FloorPlan& floorplan,
                               const PosedCamera& cam) {
	Render(floorplan, cam.Linearize(), asToon(cam.image_size()));
}

void FloorPlanRenderer::DrawOrientations(ImageRGB<byte>& canvas) {
	indoor_context::DrawOrientations(orientations(), canvas);
}

}
