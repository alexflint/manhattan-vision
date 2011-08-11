#include "common_types_vw.h"
#include "image_bundle.h"
#include "misc.h"
#include "vanishing_points.h"
#include "range_utils.tpp"
#include "quaternion.tpp"

#include <VNL/diagmatrix.h>

lazyvar<double> gvScaleX("VanPts.ScaleX");
lazyvar<double> gvScaleY("VanPts.ScaleX");

double Angle(const VecD& a, const VecD& b) {
	return acos(DotProduct(a, b)) * 180.0 / M_PI;
}

struct compare_vpts {
	bool operator()(const VanishingPoint& a, const VanishingPoint& b) const {
		return a.support > b.support;
	}
};

void DrawSkelSpot(const Vec2D& p,
									const PixelRGB<byte> color,
									ImageRGB<byte>& canvas) {
	PixelRGB<byte> white(255, 255, 255);
	DrawSpot(canvas, p, white, 5);
	DrawSpot(canvas, p, color, 4);
	DrawSpot(canvas, p, white, 3);
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

	// Find the two best-supported vanishing points
	vector<int> inds;
	compare_vpts comp;
	sorted_order(vpts.vanpts, inds, comp);

	Vec3D third = Cross3D(unscaled[inds[0]], unscaled[inds[1]]);
	Vec2D im_third = Project(vpts.lines.cond_inv * cam * third);

	DREPORT(Angle(unscaled[inds[0]], unscaled[inds[1]]));
	DREPORT(im_third);
	DREPORT(Angle(unscaled[inds[0]], third));
	DREPORT(Angle(unscaled[inds[1]], third));

	const gvar3<int> gvPad("VanPtsViz.ImagePadding");
	Vec2D offs(*gvPad, *gvPad);

	ImageRGB<byte> canvas;
	vpts.DrawHighlightViz(canvas, Vec2I(inds[0], inds[1]));
	DrawSkelSpot(im_third+offs, BrightColors::Get(5), canvas);
	WriteImage("out/third.png", canvas);

	vpts.OutputVanPointViz("out/vpts.png");

	return 0;
}
