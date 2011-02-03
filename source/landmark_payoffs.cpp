#include "landmark_payoffs.h"

#include "common_types.h"
#include "map.h"
#include "manhattan_dp.h"

namespace indoor_context {
	using namespace toon;

	void ComputeLandmarkPayoffs(const KeyFrame& frame,
															const DPGeometry& geom,
															double zfloor,
															double zceil,
															MatF& agreement_payoffs,
															MatF& occl_payoffs) {
		CHECK_NOT_NULL(frame.map);

		// Extract all the points visible in this image
		vector<Vec3> points;
		BOOST_FOREACH(const Measurement& msm, frame.measurements) {
			points.push_back(frame.map->pts[msm.point_index]);
		}

		// Compute payoffs
		ComputeLandmarkPayoffs(points, frame.image.pc(), geom,
													 zfloor, zceil,
													 agreement_payoffs, occl_payoffs);
	}

	void ComputeLandmarkPayoffs(const vector<Vec3> points,
															const PosedCamera& camera,
															const DPGeometry& geom,
															double zfloor,
															double zceil,
															MatF& agreement_payoffs,
															MatF& occl_payoffs) {
		agreement_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);
		occl_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);

		// Accumulate weights for each point
		BOOST_FOREACH(const Vec3& v, points) {
			Vec3 f = makeVector(v[0], v[1], zfloor);
			Vec3 c = makeVector(v[0], v[1], zceil);
			Vec2 im_f = geom.ImageToGrid(camera.WorldToIm(f));
			Vec2 im_c = geom.ImageToGrid(camera.WorldToIm(c));
			int x = roundi(im_f[0]);
			if (x >= 0 && x < geom.nx()) {
				int ymin = max(im_c[1]-3, 0);
				int ymax = min(im_f[1]+3, geom.ny()-1);
				for (int y = ymin; y <= ymax; y++) {
					occl_payoffs[y][x] = (y>im_c[1] && y<im_f[1]) ? 0.1 : 0.0;
					double mu = (y < (im_c[1]+im_f[1])/2) ? im_c[1] : im_f[1];
					agreement_payoffs[y][x] = Gauss1D(y, mu, 2.0);
				}
			}
		}
	}
}
