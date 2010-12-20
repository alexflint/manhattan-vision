#include "landmark_payoffs.h"

#include "common_types.h"
#include "map.h"
#include "manhattan_dp.h"

namespace indoor_context {
	void ComputeLandmarkPayoffs(const KeyFrame& frame,
															const DPGeometry& geom,
															double zfloor,
															double zceil,
															MatF& agreement_payoffs,
															MatF& occl_payoffs) {
		CHECK_NOT_NULL(frame.map);

		agreement_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);
		occl_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);

		// Accumulate weights for each point
		BOOST_FOREACH(const Measurement& msm, frame.measurements) {
			Vec3 f = frame.map->pts[msm.point_index];
			Vec3 c = frame.map->pts[msm.point_index];;
			f[2] = zfloor;
			c[2] = zceil;
			Vec2 im_f = project(geom.imageToGrid * frame.image.pc().WorldToIm(f));
			Vec2 im_c = project(geom.imageToGrid * frame.image.pc().WorldToIm(c));
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
