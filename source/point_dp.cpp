#include <boost/array.hpp>

#include "entrypoint_types.h"
#include "map.h"
#include "colors.h"
#include "canvas.h"
#include "timer.h"
#include "manhattan_dp.h"
#include "stereo_payoffs.h"
#include "geom_utils.h"
#include "bld_helpers.h"

#include "io_utils.tpp"
#include "image_utils.tpp"
#include "numeric_utils.tpp"
#include "vector_utils.tpp"

using namespace toon;
using namespace indoor_context;

Mat3 GetZHomography(const PosedCamera& pc, double z0) {
	Matrix<3,4> zcam = Zeros;
	zcam[0][0] = zcam[1][1] = zcam[2][3] = 1.0;  // zcam : [x,y,z,1] -> [x,y,1]
	return GetHomographyVia(pc.Linearize(), zcam, makeVector(0, 0, -1, z0));
}

double Gauss1D(double x, double m, double s) {
	return exp(-(x-m)*(x-m)/(2*s*s)) / (s*sqrt(2*M_PI));
}

double Gauss2D(const Vec2& x, const Vec2& m, double s) {
	return exp(-0.5 * norm_sq(m-x) / s) / (2*M_PI*sqrt(s));
}

void AddGaussian(MatF& canvas, Vec2 mean, double sigma) {
	double d = ceili(sigma*3.0);
	int x0 = Clamp<int>(mean[0]-d, 0, canvas.Cols()-1);
	int x1 = Clamp<int>(mean[0]+d, 0, canvas.Cols()-1);
	int y0 = Clamp<int>(mean[1]-d, 0, canvas.Rows()-1);
	int y1 = Clamp<int>(mean[1]+d, 0, canvas.Rows()-1);
	Vec2I v;
	for (v[1] = y0; v[1] <= y1; v[1]++) {
		float* row = canvas[v[1]];
		for (v[0] = x0; v[0] <= x1; v[0]++) {
			row[v[0]] += Gauss2D(v, mean, sigma);
		}
	}
}


template <typename T1>
string fmt(const string& f, const T1& x1) {
	return str(format(f) % x1);
}

template <typename T1, typename T2>
string fmt(const string& f, const T1& x1, const T2& x2) {
	return str(format(f) % x1 % x2);
}

template <typename T1, typename T2, typename T3>
string fmt(const string& f, const T1& x1, const T2& x2, const T3& x3) {
	return str(format(f) % x1 % x2 % x3);
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG << "Usage: "<<argv[0]<<" SEQUENCE FRAME_IDS";
		return -1;
	}

	// Input arguments
	const char* sequence = argv[1];
	const vector<int> test_ids = ParseMultiRange<int>(argv[2]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	// Get the floor and ceiling positions
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();
	Vec3 vup = map.kfs[0].pc->pose_inverse() * makeVector(0,1,0);
	if (Sign(zceil-zfloor) == Sign(vup[2])) {
		swap(zfloor, zceil);
	}

	// Payoffs
	DPPayoffs payoffs;
	ManhattanDPReconstructor recon;

	// Reconstruct each frame
	double sum_acc = 0;
	format filepat("out/frame%02d_%s.png");
	BOOST_FOREACH(int frame_id, test_ids) {
		TITLE("Reconstructing frame " << frame_id);

		// Get base frame
		const KeyFrame* frame = map.KeyFrameByIdOrDie(frame_id);
		const PosedImage& image = map.ImageByIdOrDie(frame_id);
		DPGeometry geom(&image.pc(), zfloor, zceil);

		// Get backprojectors for floor and ceiling
		Mat3 bp_floor = GetZHomography(image.pc(), zfloor);
		Mat3 bp_ceil = GetZHomography(image.pc(), zceil);

		// Initialize payoffs
		payoffs.Resize(geom.grid_size, 0);

		// Accumulate weights for each point
		BOOST_FOREACH(const Measurement& msm, frame->measurements) {
			Vec3 pt = map.pts[msm.point_index];
			Vec3 f = pt;
			Vec3 c = pt;
			f[2] = zfloor;
			c[2] = zceil;
			Vec2 im_f = project(geom.imageToGrid * image.pc().WorldToIm(f));
			Vec2 im_c = project(geom.imageToGrid * image.pc().WorldToIm(c));
			int x = roundi(im_f[0]);
			int cy = roundi(im_c[1]);
			int fy = roundi(im_f[1]);
			if (x >= 0 && x < geom.grid_size[0]) {
				for (int y = max(cy-3, 0);
						 y < min(fy+3, geom.grid_size[1]);
						 y++) {
					double bar = (y>cy && y<fy) ? 0.1 : 0.0;
					payoffs.wall_scores[0][y][x] += max(bar, max(10.0*Gauss1D(y, cy, 2.0),
																											 10.0*Gauss1D(y, fy, 2.0)));
				}
			}
		}

		// Reconstruct
		payoffs.wall_scores[1] = payoffs.wall_scores[0];
		recon.Compute(image, geom, payoffs);
		recon.OutputSolution(str(filepat % frame_id % "soln"));
		DREPORT(100.0*recon.GetAccuracy(gt_map.floorplan()));

		ImageRGB<byte> payoff_canvas;
		DrawMatrixRescaled(payoffs.wall_scores[0], payoff_canvas);
		recon.dp.DrawGridSolution(payoff_canvas);
		WriteImage(str(filepat % frame_id % "payoffs"), payoff_canvas);

		// Visualize
		BrightColors bc;
		FileCanvas canvas(str(filepat % frame_id % "proj"), image.rgb);
		BOOST_FOREACH(const Measurement& msm, frame->measurements) {
			Vec3 pt = map.pts[msm.point_index];
			Vec2 im_pt = project(image.pc().WorldToIm(pt));
			if (image.contains(asIR(im_pt))) {
				Vec3 f = pt;
				Vec3 c = pt;
				f[2] = zfloor;
				c[2] = zceil;
				Vec2 im_f = project(image.pc().WorldToIm(f));
				Vec2 im_c = project(image.pc().WorldToIm(c));
				PixelRGB<byte> color = bc.Next();
				canvas.DrawDot(im_pt, 4.0, color);
				canvas.DrawDot(im_f, 2.5, color);
				canvas.DrawDot(im_c, 2.5, color);
				canvas.StrokeLine(im_f, im_c, color);
			}
		}
	}

	return 0;
}
