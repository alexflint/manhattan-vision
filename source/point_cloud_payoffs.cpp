#include "point_cloud_payoffs.h"

#include "common_types.h"
#include "map.h"
#include "manhattan_dp.h"

#include "canvas.tpp"

namespace indoor_context {
	using namespace toon;

	lazyvar<double> gvAgreeSigma("LandmarkPayoffs.AgreeSigma");

	// These cannot be gvars because they are used as template parameters
	static const int kCutoff = 6;  // should be > gvAgreeSigma*3
	static const int kWin = 13;  // should be 2 * kCutoff + 1

	void PointCloudPayoffs::Compute(const vector<Vec3>& points,
																const PosedCamera& camera,
																const DPGeometry& geom,
																double zf,
																double zc) {
		input_points = &points;
		input_camera = &camera;
		geometry = geom;
		zfloor = zf;
		zceil = zc;

		CHECK_GE(kCutoff, *gvAgreeSigma*2)
			<< "After changing PointCloudPayoffs.AgreeSigma, you must also change constants at top of landmark_payoffs.cpp";

		// Resize matrices
		agreement_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);
		occlusion_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);

		// Cache gaussian for speed
		Vector<kWin> gauss1d = Zeros;
		for (int i = 0; i < gauss1d.size(); i++) {
			gauss1d[i] = Gauss1D(i-kCutoff, 0, *gvAgreeSigma);
		}

		// Count the number of points that project to each point in the grid
		MatI proj_counts(geom.grid_size[1], geom.grid_size[0], 0);
		BOOST_FOREACH(const Vec3& v, points) {
			Vec3 f = makeVector(v[0], v[1], zfloor);
			Vec3 c = makeVector(v[0], v[1], zceil);
			Vec2 grid_f = geom.ImageToGrid(camera.WorldToIm(f));
			Vec2 grid_c = geom.ImageToGrid(camera.WorldToIm(c));
			CHECK_LE(grid_c[1], grid_f[1]) << "Ceiling points should project above floor in grid";

			for (int i = 0; i < 2; i++) {
				Vec2 grid_pt = i ? grid_f : grid_c;
				Vec2I grid_p = makeVector(roundi(grid_pt[0]), roundi(grid_pt[1]));
				if (grid_p[0] >= 0 && grid_p[0] < geom.grid_size[0] &&
						grid_p[1] >= 0 && grid_p[1] < geom.grid_size[1]) {
					proj_counts[ grid_p[1] ][ grid_p[0] ] += (i ? -1 : 1);  // floor points count for -1, ceiling +1
				}
			}
		}

		for (int x = 0; x < geom.grid_size[0]; x++) {
			// Initialize counts
			Vector<kWin> dist_counts = Zeros;
			for (int i = kCutoff; i < kWin; i++) {
				dist_counts[i] = abs(proj_counts[i-kCutoff][x]);
			}

			// Calculate agreement payoffs
			int agree_counts = 0;
			for (int y = 0; y < geom.grid_size[1]; y++) {
				agree_counts += proj_counts[y][x];  // sign will cause corresponding floor/ceiling projs to cancel

				// Update the last window
				if (y+kWin < geom.grid_size[1]) {
					dist_counts[kWin-1] = abs(proj_counts[y+kCutoff][x]);
				} else {
					dist_counts[kWin-1] = 0;
				}

				// Compute payoffs
				agreement_payoffs[y][x] = dist_counts * gauss1d;
				occlusion_payoffs[y][x] = agree_counts;

				// Shift left
				dist_counts.slice<0,kWin-1>() = dist_counts.slice<1,kWin-1>();
			}
		}
	}

	void PointCloudPayoffs::ComputeSlow(const KeyFrame& frame,
																const DPGeometry& geom,
																double zfloor,
																double zceil) {
		CHECK_NOT_NULL(frame.map);

		// Extract all the points visible in this image
		vector<Vec3> points;
		BOOST_FOREACH(const Measurement& msm, frame.measurements) {
			points.push_back(frame.map->pts[msm.point_index]);
		}

		// Compute payoffs
		ComputeSlow(points, frame.image.pc(), 
						geom, zfloor, zceil);
	}

	void PointCloudPayoffs::ComputeSlow(const vector<Vec3>& points,
																		const PosedCamera& camera,
																		const DPGeometry& geom,
																		double zf,
																		double zc) {
		input_points = &points;
		input_camera = &camera;
		geometry = geom;
		zfloor = zf;
		zceil = zc;

		agreement_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);
		occlusion_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);

		/*VecI col0_counts(geom.grid_size[1], 0);
		Vector<> dist_from_09(kWin);
		dist_from_09 = Zeros;*/

		// Accumulate weights for each point
		BOOST_FOREACH(const Vec3& v, points) {
			Vec3 f = makeVector(v[0], v[1], zfloor);
			Vec3 c = makeVector(v[0], v[1], zceil);
			Vec2 grid_f = geom.ImageToGrid(camera.WorldToIm(f));
			Vec2 grid_c = geom.ImageToGrid(camera.WorldToIm(c));
			CHECK_LE(grid_c[1], grid_f[1]);
			CHECK_EQ_TOL(grid_f[0], grid_c[0], 1.0);

			//canvas.DrawDot(grid_f, 2.0, Colors::alpha(0.1, Colors::red()));
			//canvas.DrawDot(grid_c, 2.0, Colors::alpha(0.1, Colors::blue()));
			int x = roundi(grid_f[0]);
			if (x >= 0 && x < geom.nx()) {
				/*Vec2I grid_p = makeVector(roundi(grid_f[0]), roundi(grid_f[1]));
				if (grid_p[0] == 0 &&
						grid_p[1] >= 0 && grid_p[1] < geom.grid_size[1]) {
					if (grid_p == makeVector(0,1)) DREPORT(grid_p, grid_f);
					col0_counts[ grid_p[1] ]++;
				}

				grid_p = makeVector(roundi(grid_c[0]), roundi(grid_c[1]));
				if (grid_p[0] == 0 &&
						grid_p[1] >= 0 && grid_p[1] < geom.grid_size[1]) {
					if (grid_p == makeVector(0,1)) DREPORT(grid_p, grid_c);
					col0_counts[ grid_p[1] ]++;
					}*/

				/*if (x==0 && grid_c[1] >= 0 && grid_c[1] < geom.ny()) {
					col0_counts[floori(grid_c[1])]++;
				}
				if (x==0 && grid_f[1] >= 0 && grid_f[1] < geom.ny()) {
					col0_counts[floori(grid_f[1])]++;
					}*/

				int ymin = max(grid_c[1]-kCutoff, 0);
				int ymax = min(grid_f[1]+kCutoff, geom.ny()-1);
				for (int y = ymin; y <= ymax; y++) {
					occlusion_payoffs[y][x] += (y>grid_c[1] && y<grid_f[1]) ? 1.0 : 0.0;
					int mu = roundi(y < 0.5*(grid_c[1]+grid_f[1]) ? grid_c[1] : grid_f[1]);
					agreement_payoffs[y][x] += Gauss1D(y, mu, 2.0);
					/*if (x == 0 && y == 9) {
						dist_from_09[mu-y+kCutoff]++;
						}*/
				}
			}
		}

		//DREPORT(col0_counts, dist_from_09);
	}

	void PointCloudPayoffs::OutputProjectionViz(const ImageBundle& image,
																						const string& path) {
		FileCanvas canvas(path, image.rgb);
		
		for (int i = 0; i < 200; i++) {
			int ii = rand() % input_points->size();
			Vec3 v = (*input_points)[ii];
			Vec3 f = makeVector(v[0], v[1], zfloor);
			Vec3 c = makeVector(v[0], v[1], zceil);
			/*Vec2 grid_v = geometry.ImageToGrid(input_camera->WorldToIm(v));
			Vec2 grid_f = geometry.ImageToGrid(input_camera->WorldToIm(f));
			Vec2 grid_c = geometry.ImageToGrid(input_camera->WorldToIm(c));*/
			Vec2 grid_v = project(input_camera->WorldToIm(v));
			Vec2 grid_f = project(input_camera->WorldToIm(f));
			Vec2 grid_c = project(input_camera->WorldToIm(c));

			canvas.StrokeLine(grid_v, grid_f, Colors::black());
			canvas.StrokeLine(grid_v, grid_c, Colors::black());
			canvas.DrawDot(grid_v, 3.0, Colors::white());
			canvas.DrawDot(grid_f, 3.0, Colors::red());
			canvas.DrawDot(grid_c, 3.0, Colors::blue());
		}
	}
}
