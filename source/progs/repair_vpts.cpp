#include "common_types_vw.h"
#include "image_bundle.h"
#include "misc.h"
#include "vanishing_points.h"

#include "quaternion.tpp"

#include <VNL/diagmatrix.h>

lazyvar<double> gvScaleX("VanPts.ScaleX");
lazyvar<double> gvScaleY("VanPts.ScaleX");

double Angle(const VecD& a, const VecD& b) {
	return acos(DotProduct(a, b)) * 180.0 / M_PI;
}

int main(int argc, char **argv) {
	InitVars(argc, argv, "common.cfg");

	// Read the images and find vanishing points
	ImageBundle image(argv[1]);
	VanishingPointsEM vpts(image);

	if (vpts.vanpts.size() < 3) {
		DLOG << "Less than three vanpts detected, cannot repair";
		return -1;
	}

	// Construct camera calibration matrix
	DiagMatrix<double> cam(Vec3D(1.0 / *gvScaleX, 1.0 / *gvScaleY, 1.0));
	DiagMatrix<double> caminv(Vec3D(*gvScaleX, *gvScaleY, 1.0));

	// Compute un-scaled vanishing points
	vector<Vec3D> unscaled;
	for (int j = 0; j < vpts.vanpts.size(); j++) {
		unscaled.push_back((caminv * vpts.vanpts[j].pt_cond).Normalize());
	}

	// Find best-fit triplet
	double besterr = INFINITY;
	Vec3I bestinds;
	QtrnD bestq;
	for (int a = 0; a < vpts.vanpts.size(); a++) {
		for (int b = a+1; b < vpts.vanpts.size(); b++) {
			for (int c = b+1; c < vpts.vanpts.size(); c++) {
				DLOG <<a<<","<<b<<","<<c<<":";
				SCOPED_INDENT;

				QtrnD q = FitQtrn(unscaled[a], unscaled[b], unscaled[c]);

				Vec3I inds(a, b, c);
				double err = 0.0;
				for (int j = 0; j < 3; j++) {
					DLOG << "axis " << j << ":";
					SCOPED_INDENT;
					Vec3D ej(0.0);
					ej[j] = 1.0;
					Vec3D est = QtrnTransform(ej, q);
					double d = AngularError(est, unscaled[inds[j]]);
					err = max(err, d);
					DREPORT(Project(unscaled[inds[j]]));
					DREPORT(ej);
					DREPORT(Project(est));
				}

				DREPORT(err);
				if (err < besterr) {
					besterr = err;
					bestinds = inds;
					bestq = q;
				}
			}
		}
	}

	DLOG << "Best error: " << besterr;

	vector<Vec3D> estimates;
	for (int j = 0; j < 3; j++) {
		Vec3D ej(0.0);
		ej[j] = 1.0;
		estimates.push_back(QtrnTransform(ej, bestq));
	}

	ImageRGB<byte> canvas;
	vpts.DrawHighlightViz(canvas, bestinds);
	for (int j = 0; j < 3; j++) {
		DLOG << "Axis " << j;
		SCOPED_INDENT;

		Vec3D est = estimates[j];
		Vec2D proj_est = Project(vpts.lines.cond_inv * cam * est);

		Vec3D orig = unscaled[bestinds[j]];
		Vec2D proj_orig = Project(vpts.lines.cond_inv * cam * orig);
		Vec2D orig_obs = Project(vpts.vanpts[bestinds[j]].pt);

		Vec3D next = unscaled[bestinds[(j+1)%3]];

		DREPORT(orig);
		DREPORT(est);
		DREPORT(orig_obs);
		DREPORT(proj_orig);
		DREPORT(proj_est);
		DREPORT(Angle(orig, est));
		DREPORT(Angle(orig, next));



		const PixelRGB<byte> color = BrightColors::Get(bestinds[j]);
		PixelRGB<byte> white(255, 255, 255);
		DrawSpot(canvas, proj_est, white, 5);
		DrawSpot(canvas, proj_est, color, 4);
		DrawSpot(canvas, proj_est, white, 3);
	}

	WriteImage("out/repaired.png", canvas);
	vpts.OutputVanPointViz("out/vpts.png");

	return 0;
}
