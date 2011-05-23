#include "point_cloud_payoffs.h"

#include "common_types.h"
#include "map.h"
#include "manhattan_dp.h"
#include "colors.h"
#include "gaussian.h"

#include "canvas.tpp"

namespace indoor_context {
	using namespace toon;

	lazyvar<double> gvAgreeSigma("LandmarkPayoffs.AgreeSigma");

	// These cannot be gvars because they are used as template parameters
	//static const int gaussian_cutoff = 6;  // should be > gvAgreeSigma*3
	//static const int gaussian_window = 13;  // should be 2*cutoff+1

	PointCloudPayoffs::PointCloudPayoffs() : agreement_sigma(*gvAgreeSigma) {
	}

	void PointCloudPayoffs::Compute(const vector<Vec3>& points,
																	const PosedCamera& camera,
																	const DPGeometryWithScale& geom) {
		input_points = &points;
		input_camera = &camera;
		geometry = geom;

		const int gaussian_cutoff = ceili(sqrt(agreement_sigma) * 3.);  // cutoff after 3 standard deviations
		const int gaussian_window = gaussian_cutoff*2+1;

		CHECK_GE(gaussian_cutoff, sqrt(agreement_sigma)*2)
			<< "After changing PointCloudPayoffs.AgreeSigma, you must also change constants at top of landmark_payoffs.cpp";

		// Resize matrices
		agreement_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);
		occlusion_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);

		// Cache gaussian for speed
		Vector<> gauss1d(gaussian_window);
		gauss1d = Zeros;
		for (int i = 0; i < gauss1d.size(); i++) {
			gauss1d[i] = Gauss1D(i-gaussian_cutoff, 0, agreement_sigma);
		}

		// Count the number of points that project to each point in the grid
		MatI proj_counts(geom.grid_size[1], geom.grid_size[0], 0);
		BOOST_FOREACH(const Vec3& v, points) {
			Vec3 f = makeVector(v[0], v[1], geometry.zfloor);
			Vec3 c = makeVector(v[0], v[1], geometry.zceil);
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
			Vector<> dist_counts(gaussian_window);
			dist_counts = Zeros;
			for (int i = gaussian_cutoff; i < gaussian_window; i++) {
				dist_counts[i] = abs(proj_counts[i-gaussian_cutoff][x]);
			}

			// Calculate agreement payoffs
			int agree_counts = 0;
			for (int y = 0; y < geom.grid_size[1]; y++) {
				agree_counts += proj_counts[y][x];  // sign will cause corresponding floor/ceiling projs to cancel

				// Update the last window
				if (y+gaussian_window < geom.grid_size[1]) {
					dist_counts[gaussian_window-1] = abs(proj_counts[y+gaussian_cutoff][x]);
				} else {
					dist_counts[gaussian_window-1] = 0;
				}

				// Compute payoffs
				agreement_payoffs[y][x] = dist_counts * gauss1d;
				occlusion_payoffs[y][x] = agree_counts;

				// Shift left
				dist_counts.slice(0,gaussian_window-1) = dist_counts.slice(1,gaussian_window-1);
			}
		}
	}

	void PointCloudPayoffs::ComputeSlow(const Frame& frame,
																			const DPGeometryWithScale& geom) {
		CHECK_NOT_NULL(frame.map);

		// Extract all the points visible in this image
		vector<Vec3> points;
		frame.GetMeasuredPoints(points);

		// Compute payoffs
		ComputeSlow(points, frame.image.pc(), geom);
	}

	void PointCloudPayoffs::ComputeSlow(const vector<Vec3>& points,
																			const PosedCamera& camera,
																			const DPGeometryWithScale& geom) {
		input_points = &points;
		input_camera = &camera;
		geometry = geom;

		const int gaussian_cutoff = agreement_sigma * 2;
		const int gaussian_window = gaussian_cutoff*2+1;

		agreement_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);
		occlusion_payoffs.Resize(geom.grid_size[1], geom.grid_size[0], 0);

		// Accumulate weights for each point
		BOOST_FOREACH(const Vec3& v, points) {
			Vec3 f = makeVector(v[0], v[1], geometry.zfloor);
			Vec3 c = makeVector(v[0], v[1], geometry.zceil);
			Vec2 grid_f = geom.ImageToGrid(camera.WorldToIm(f));
			Vec2 grid_c = geom.ImageToGrid(camera.WorldToIm(c));
			CHECK_LE(grid_c[1], grid_f[1]);
			CHECK_EQ_TOL(grid_f[0], grid_c[0], 1.0);

			int x = roundi(grid_f[0]);
			if (x >= 0 && x < geom.nx()) {
				int ymin = max(grid_c[1]-gaussian_cutoff, 0);
				int ymax = min(grid_f[1]+gaussian_cutoff, geom.ny()-1);
				for (int y = ymin; y <= ymax; y++) {
					occlusion_payoffs[y][x] += (y>grid_c[1] && y<grid_f[1]) ? 1.0 : 0.0;
					int mu = roundi(y < 0.5*(grid_c[1]+grid_f[1]) ? grid_c[1] : grid_f[1]);
					agreement_payoffs[y][x] += Gauss1D(y, mu, 2.0);
				}
			}
		}
	}

	void PointCloudPayoffs::OutputProjectionViz(const ImageBundle& image,
																							const string& path) {
		FileCanvas canvas(path, image.rgb);
		
		for (int i = 0; i < 200; i++) {
			int ii = rand() % input_points->size();
			Vec3 v = (*input_points)[ii];
			Vec3 f = makeVector(v[0], v[1], geometry.zfloor);
			Vec3 c = makeVector(v[0], v[1], geometry.zceil);
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
