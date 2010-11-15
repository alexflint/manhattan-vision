/*
 * simple_render.h
 *
 *  Created on: 6 Jul 2010
 *      Author: alexf
 */

#include "simple_renderer.h"
#include "common_types.h"
#include "geom_utils.h"

#include "fill_polygon.tpp"
#include "clipping3d.tpp"

namespace indoor_context {

static const double kExtent = 1e+3;  // extent of horizontal surfaces for RenderHorizSurface

SimpleRenderer::SimpleRenderer() {
}

SimpleRenderer::SimpleRenderer(const PosedCamera& camera) {
	Configure(camera);
}

SimpleRenderer::SimpleRenderer(const toon::Matrix<3,4>& camera, Vec2I viewport) {
	Configure(camera, viewport);
}

void SimpleRenderer::Configure(const PosedCamera& camera) {
	Configure(camera.Linearize(), asToon(camera.image_size()));
}

void SimpleRenderer::Configure(const toon::Matrix<3,4>& camera, Vec2I viewport) {
	viewport_ = viewport;
	camera_ = camera;
	framebuffer_.Resize(viewport[1], viewport[0]);
	depthbuffer_.Resize(viewport[1], viewport[0]);
	Clear(0);
}

bool SimpleRenderer::Render(const Vec2& p, const Vec2& q, const Vec2& r, int label) {
	return Render(unproject(p), unproject(q), unproject(r), label);
}

bool SimpleRenderer::Render(const Vec3& p, const Vec3& q, const Vec3& r, int label) {
	// Do 3D clipping
	Vec3 vs[] = {p,q,r};
	vector<Vec3> clipped;
	ClipToFrustrum(array_range(vs,3), camera_, viewport_, back_inserter(clipped));

	// Project into the camera
	vector<Vec3> projected;
	BOOST_FOREACH(const Vec3& v, clipped) {
		projected.push_back(camera_ * unproject(v));
	}

	// Compute the triangle scanlines
	int y0;
	vector<pair<int, int> > scanlines;
	ComputeFillScanlines(projected, viewport_, y0, scanlines);

	// Set up the depth equation
	Vec3 nrm = (p-q)^(p-r);
	Vec4 plane = concat(nrm, -nrm*p);
	Vec3 depth_eqn = PlaneToDepthEqn(camera_, plane);

	// Do the rendering
	bool affected = false;
	for (int i = 0; i < scanlines.size(); i++) {
		// Pre-compute the first bit of the depth equation
		double depth_base = depth_eqn * makeVector(0, y0+i, 1);
		double depth_coef = depth_eqn[0];

		// Fill the row
		double* depth_row = depthbuffer_[y0+i];
		int* label_row = framebuffer_[y0+i];
		for (int x = scanlines[i].first; x <= scanlines[i].second; x++) {
			double depth = 1.0 / (depth_base + depth_coef*x);  // see PlaneToDepthEqn in geom_utils.h
			if (depth < depth_row[x]) {
				depth_row[x] = depth;
				label_row[x] = label;
				affected = true;
			}
		}
	}

	return affected;
}

bool SimpleRenderer::RenderInfinitePlane(double z0, int label) {
	Vec3 nw = makeVector(-kExtent, -kExtent, z0);
	Vec3 ne = makeVector(kExtent, -kExtent, z0);
	Vec3 se = makeVector(kExtent, kExtent, z0);
	Vec3 sw = makeVector(-kExtent, kExtent, z0);
	Render(nw, ne, se, label);
	Render(nw, sw, se, label);
}

void SimpleRenderer::Clear(int bg) {
	framebuffer_.Fill(bg);
	depthbuffer_.Fill(INFINITY);
}

}
