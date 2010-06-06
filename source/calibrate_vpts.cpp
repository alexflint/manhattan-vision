#include <boost/ptr_container/ptr_vector.hpp>

#include <VW/Image/imagecopy.h>
#include <VNL/vector.tpp>
#include <VNL/Algo/matrixinverse.h>

#include "common_types_vw.h"
#include "timer.h"
#include "vanishing_points.h"
#include "image_bundle.h"
#include "range_utils.tpp"
#include "vpt_calib.h"
#include "vanishing_points.h"
#include "vars.h"

using namespace indoor_context;

const lazyvar<int> gvNumIters("ConsVanPts.NumIterations");
const lazyvar<double> gvVoteThresh("ConsVanPts.VoteThreshold");

PixelRGB<byte> kElimColor(0, 0, 0);

double EstimateF(const Vec3D& a, const Vec3D& b) {
	return sqrt((-a[0]*b[0] - a[1]*b[1]) / (a[2]*b[2]));
}

int main(int argc, char **argv) {
	InitVars(argc, argv, "common.cfg");
	if (argc != 5) {
		DLOG << "Usage: "<<argv[0]<<" IMAGE INDEX1 INDEX2 INDEX3";
		return -1;
	}
	const char* inputfile = argv[1];
	const VecI inds(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));

	// Read the images and find vanishing points
	ImageBundle image(argv[1]);
	VanishingPointsEM vpts(image);
	const Vec3D& v1 = vpts.vanpts[inds[0]].pt_cond;
	const Vec3D& v2 = vpts.vanpts[inds[1]].pt_cond;
	const Vec3D& v3 = vpts.vanpts[inds[2]].pt_cond;

	DREPORT(v1);
	DREPORT(v2);
	DREPORT(v3);

	Vec3D v2d = v2+Vec3D(0.005, 0.005, 0.0);
	DREPORT(v2d);
	DREPORT(Project(vpts.lines.cond_inv*v2));
	DREPORT(Project(vpts.lines.cond_inv*v2d));

	DREPORT(EstimateF(v1, v2));
	DREPORT(EstimateF(v1, v2d));
	DREPORT(EstimateF(v1, v3));
	DREPORT(EstimateF(v2, v3));

	DREPORT(Project(vpts.lines.cond * Vec3D(0, 0, 1)));
	DREPORT(Project(vpts.lines.cond * Vec3D(160, 120, 1)));
	DREPORT(Project(vpts.lines.cond * Vec3D(320, 240, 1)));

	DREPORT(Project(vpts.lines.cond_inv * Vec3D(0, 0, 1)));

	DREPORT(Project(v1));
	DREPORT(Project(v2));
	DREPORT(Project(v3));

	// Estimate calibration parameters
	vector<pair<Vec3D,Vec3D> > data;
	data.push_back(make_pair(v1, v2));
	data.push_back(make_pair(v1, v3));
	MatrixFixed<3,3,double> cam;
	FitScaleModel(data, cam);
	
	vpts.OutputVanPointViz("out/vpts.png");

	DLOG << "Estimated camera matrix:";
	DLOG << cam;
	INDENTED {
		DLOG << "Scale_x = " << sqrt(cam[0][0]);
		DLOG << "Scale_y = " << sqrt(cam[1][1]);
	}

	return 0;
}
