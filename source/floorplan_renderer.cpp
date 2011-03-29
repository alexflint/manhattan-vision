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

void FloorPlanRenderer::Configure(const PosedCamera& camera) {
	renderer_.Configure(camera);
}

void FloorPlanRenderer::Configure(const Matrix<3,4>& camera,
																	const Vec2I& viewport) {
	renderer_.Configure(camera, viewport);
}

void FloorPlanRenderer::Reset(double zfloor, double zceil) {
	// Clear vectors
	walls_.clear();
	wall_labels_.clear();
	renderer_.Clear(-1);  // Need to reset depth

	// Render the floor and ceiling planes.
	renderer_.RenderInfinitePlane(zfloor, kVerticalAxis);
	renderer_.RenderInfinitePlane(zceil, kVerticalAxis);
}

int FloorPlanRenderer::GetWallOrientation(const Vec2& u, const Vec2& v) {
	return abs(u[0]-v[0]) > abs(u[1]-v[1]) ? 1 : 0;
}

void FloorPlanRenderer::RenderWall(const Vec2& u, const Vec2& v, double zfloor, double zceil) {
	// Construct the quad
	Polygon<4> wall;
	wall.verts[0] = concat(u, zceil);
	wall.verts[1] = concat(v, zceil);
	wall.verts[2] = concat(v, zfloor);
	wall.verts[3] = concat(u, zfloor);

	// Label surfaces by their normal direction
	int label = GetWallOrientation(u, v);
	renderer_.Render(wall.verts[0], wall.verts[1], wall.verts[2], label);
	renderer_.Render(wall.verts[0], wall.verts[2], wall.verts[3], label);

	// Add to list
	walls_.push_back(wall);
	wall_labels_.push_back(label);
}

void FloorPlanRenderer::Render(const proto::FloorPlan& fp) {
	Reset(fp.zfloor(), fp.zceil());
	int nv = fp.vertices_size();
	for (int i = 0; i < nv; i++) {
		Vec2 u = asToon(fp.vertices(i));
		Vec2 v = asToon(fp.vertices((i+1)%nv));
		if (!isnan(u) && !isnan(v)) {
			RenderWall(u, v, fp.zfloor(), fp.zceil());
		}
 	}
}

void FloorPlanRenderer::Render(const proto::Model& model) {
	Reset(model.zfloor(), model.zceil());
	BOOST_FOREACH(const proto::Polyline& line, model.segments()) {
		for (int i = 0; i+1 < line.vertices_size(); i++) {
			Vec2 u = asToon(line.vertices(i));
			Vec2 v = asToon(line.vertices(i+1));
			//DREPORT(u,v);
			if (!isnan(u) && !isnan(v)) {
				RenderWall(u, v, model.zfloor(), model.zceil());
			}
		}
	}
}

void FloorPlanRenderer::DrawOrientations(ImageRGB<byte>& canvas) {
	indoor_context::DrawOrientations(orientations(), canvas);
}

}
