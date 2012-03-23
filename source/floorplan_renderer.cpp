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
#include "geom_utils.h"
#include "protobuf_utils.h"

#include "image_utils.tpp"
#include "format_utils.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "io_utils.tpp"

namespace indoor_context {
	using namespace toon;

	FloorPlanRenderer::FloorPlanRenderer() {
	}

	void FloorPlanRenderer::Configure(const PosedCamera& camera) {
		renderer_.Configure(camera);
		orientations_.Resize(camera.ny(), camera.nx());
	}

	void FloorPlanRenderer::Configure(const Matrix<3,4>& camera,
																		const Vec2I& viewport) {
		renderer_.Configure(camera, viewport);
		orientations_.Resize(viewport[1], viewport[0]);
	}

	void FloorPlanRenderer::Reset(double zfloor, double zceil) {
		// Clear vectors
		walls_.clear();
		wall_orients_.clear();
		renderer_.Clear(-1);  // Need to reset depth

		// Render the floor and ceiling planes.
		// Will translate -1 to kVerticalAxis later
		renderer_.RenderInfinitePlane(zfloor, -1);
		renderer_.RenderInfinitePlane(zceil, -2);
		orientations_.Fill(kVerticalAxis);

		// Store these for BackProject
		zfloor_ = zfloor;
		zceil_ = zceil;
	}

	int FloorPlanRenderer::GetWallOrientation(const Vec2& u, const Vec2& v) {
		return abs(u[0]-v[0]) > abs(u[1]-v[1]) ? 1 : 0;
	}

	void FloorPlanRenderer::Render(const proto::FloorPlan& fp) {
		Reset(fp.zfloor(), fp.zceil());
		int nv = fp.vertices_size();

		// Render each wall as a quad
		for (int i = 0; i < nv; i++) {
			Vec2 u = asToon(fp.vertices(i));
			Vec2 v = asToon(fp.vertices((i+1)%nv));
			if (!isnan(u) && !isnan(v)) {
				// Construct the quad
				Polygon<4> wall;
				wall.verts[0] = concat(u, fp.zceil());
				wall.verts[1] = concat(v, fp.zceil());
				wall.verts[2] = concat(v, fp.zfloor());
				wall.verts[3] = concat(u, fp.zfloor());

				// Label surfaces by their index. Will translate to orientations later.
				renderer_.Render(wall.verts[0], wall.verts[1], wall.verts[2], walls_.size());
				renderer_.Render(wall.verts[0], wall.verts[2], wall.verts[3], walls_.size());

				// Add to list
				walls_.push_back(wall);
				wall_orients_.push_back(GetWallOrientation(u, v));
			}
		}

		// Translate wall labels to orientations
		for (int y = 0; y < orientations_.Rows(); y++) {
			int* orientrow = orientations_[y];
			const int* labelrow = renderer_.framebuffer()[y];
			for (int x = 0; x < orientations_.Cols(); x++) {
				if (labelrow[x] < 0) {
					orientrow[x] = kVerticalAxis;
				} else {
					orientrow[x] = wall_orients_[labelrow[x]];
				}
			}
		}
	}

	int FloorPlanRenderer::GetWallIndexAt(const Vec2I& pixel) const {
		CHECK(GetOrientationAt(pixel) != kVerticalAxis)
			<< "Attempted to get wall index at a non-wall pixel";
		return renderer_.framebuffer()[ pixel[1] ][ pixel[0] ];
	}

	int FloorPlanRenderer::GetOrientationAt(const Vec2I& pixel) const {
		return orientations_[ pixel[1] ][ pixel[0] ];
	}

	Polygon<4> FloorPlanRenderer::GetWallAt(const Vec2I& pixel) const {
		return walls_[GetWallIndexAt(pixel)];
	}

	Vec3 FloorPlanRenderer::BackProject(const Vec2& v) const {
		// Find the plane equation for the object that projects to this pixel
		Vec4 plane;
		int label = renderer_.framebuffer()[ roundi(v[1]) ][ roundi(v[0]) ];
		if (label < 0) {
			double z = label == -1 ? zfloor_ : zceil_;
			plane = makeVector(0, 0, 1, -z);
		} else {
			Polygon<4> wall = GetWallAt(v);
			Vec3 nrm = (wall.verts[1]-wall.verts[0]) ^ (wall.verts[2]-wall.verts[0]);
			plane = concat(nrm, -nrm*wall.verts[0]);
		}
		
		// Back-project onto the plane
		return IntersectRay(unproject(v), renderer_.camera(), plane);
	}

	void FloorPlanRenderer::DrawOrientations(ImageRGB<byte>& canvas) {
		indoor_context::DrawOrientations(orientations(), canvas);
	}
}
