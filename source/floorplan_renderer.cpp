/*
 * floorplan_renderer.cpp
 *
 *  Created on: 1 Jun 2010
 *      Author: alexf
 */

#include "floorplan_renderer.h"

#include <boost/foreach.hpp>

#include "common_types.h"
#include "map.pb.h"
#include "camera.h"
#include "colored_points.h"
#include "viewer3d.h"

#include "math_utils.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
using namespace toon;

FloorplanRenderer::FloorplanRenderer() {
}

FloorplanRenderer::FloorplanRenderer(const proto::TruthedMap& tru_map) {
	Configure(tru_map);
}

void FloorplanRenderer::Configure(const proto::TruthedMap& tru_map) {
	const proto::FloorPlan& fp = tru_map.floorplan();

	planes.push_back(makeVector(0.0, 0.0, 1.0, -fp.zfloor()));
	plane_orients.push_back(2);
	planes.push_back(makeVector(0.0, 0.0, 1.0, -fp.zceil()));
	plane_orients.push_back(2);

	Vec4 w = Zeros;
	for (int i = 0, n = fp.vertices_size(); i < n; i++) {
		Vec3 u = concat(asToon(fp.vertices(i)), 0.0);
		Vec3 v = concat(asToon(fp.vertices((i+1)%n)), 0.0);
		if (u == v || isnan(u) || isnan(v)) continue;
		w.slice<0,3>() = (u-v)^GetAxis<3>(2);
		w[3] = w.slice<0,3>()*u;
		planes.push_back(w);
		plane_orients.push_back(abs(u[0]-v[0]) > abs(u[1]-v[1]) ? 0 : 1);
	}
}

void FloorplanRenderer::PredictSurfs(const PosedCamera& pc, MatI& orients) {
}

void FloorplanRenderer::PredictOrients(const PosedCamera& pc, MatI& orients) {
	orients.Resize(pc.im_size().y, pc.im_size().x);
	MatF depth(pc.im_size().y, pc.im_size().x);

	vector<Vec3> depth_eqns;
	Matrix<3,4> cam = as_matrix(pc.pose);
	BOOST_FOREACH(const Vec4& plane, planes) {
		depth_eqns.push_back(PlaneToDepthEqn(cam, plane));
	}
	DREPORT(pc.pose, cam);

	ColoredPoints pts;

	Vec3 p = Ones, ret;
	for (int y = 0; y < orients.Rows(); y++) {
		p[1] = y;
		int* orientrow = orients[y];
		for (int x = 0; x < orients.Cols(); x++) {
			p[0] = x;
			double mindepth = INFINITY;  // initialize to z-near
			ret = pc.ImToRet(p);
			orientrow[x] = -1;  // is possible to reach here if all planes behind camera
			int selected = -1;
			for (int i = 0; i < depth_eqns.size(); i++) {
				double depth = 1.0/(depth_eqns[i]*ret);
				if (x%5==0 && y%5==0) {
					pts.Add(IntersectRay(ret, cam, planes[i]), Colors::grey(depth));// BrightColors::Get(i));
					/*CHECK_TOL(depth, GetPlaneDepth(ret, cam, planes[i]), 1e-5)
							<< "x="<<x<<" y="<<y<<" ret="<<ret<<" planes[i]="<<planes[i]<<" i="<<i;*/
				}
				if (depth > 1e-6 && depth < mindepth) {
					mindepth = depth;
					selected = pts.vs.size();
					orientrow[x] = plane_orients[i];
				}
			}
			//pts.vs[selected].second = Colors::white();
			depth[y][x] = mindepth;
		}
	}

	Viewer3D v;
	v.Add(pts);
	v.Run();

	WriteMatrixImageRescaled("out/depth.png", depth);
}


}
