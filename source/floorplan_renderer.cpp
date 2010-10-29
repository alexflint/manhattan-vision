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

namespace indoor_context {
using namespace toon;

FloorPlanRenderer::FloorPlanRenderer() {
}

void FloorPlanRenderer::RenderInternal(const proto::FloorPlan& fp,
                                       const Matrix<3,4>& cam,
                                       const Vec2I& viewport) {
	int nv = fp.vertices_size();
	renderer_.Configure(cam, viewport);
	renderer_.Clear(2);  // initialize with the vertical label
	for (int i = 0; i < nv; i++) {
		Vec2 u = asToon(fp.vertices(i));
		Vec2 v = asToon(fp.vertices((i+1)%nv));
		if (isnan(u) || isnan(v)) continue;

		Vec3 p = concat(u, fp.zceil());
		Vec3 q = concat(v, fp.zceil());
		Vec3 r = concat(v, fp.zfloor());
		Vec3 s = concat(u, fp.zfloor());

		// label surfaces by their normal direction...
		int label = abs(u[0]-v[0]) > abs(u[1]-v[1]) ? 1 : 0;
		renderer_.Render(p, q, r, label);
		renderer_.Render(p, r, s, label);
	}
}

// Render from a particular view
void FloorPlanRenderer::Render(const proto::FloorPlan& floorplan,
                               const Matrix<3,4>& cam,
                               const Vec2I& viewport,
                               ImageRGB<byte>& canvas) {
	RenderInternal(floorplan, cam, viewport);
	DrawOrientations(renderer_.framebuffer(), canvas);
}

void FloorPlanRenderer::Render(const proto::FloorPlan& floorplan,
                               const PosedCamera& cam,
                               ImageRGB<byte>& canvas) {
	Render(floorplan,
				 cam.Linearize(),
				 asToon(cam.image_size()),
				 canvas);
}

// Render orientations
void FloorPlanRenderer::RenderOrients(const proto::FloorPlan& floorplan,
                                      const Matrix<3,4>& cam,
                                      const Vec2I& viewport,
                                      MatI& orients) {
	RenderInternal(floorplan, cam, viewport);
	orients = renderer_.framebuffer();
}

void FloorPlanRenderer::RenderOrients(const proto::FloorPlan& floorplan,
                                      const PosedCamera& cam,
                                      MatI& orients) {
	RenderOrients(floorplan,
								cam.Linearize(),
								asToon(cam.image_size()),
								orients);
}

}
