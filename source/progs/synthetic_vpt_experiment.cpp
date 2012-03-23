#include <stdlib.h>

#include <fstream>

#include <boost/format.hpp>

#include <TooN/so3.h>

#include "common_types.h"
#include "canvas.h"
#include "colors.h"
#include "line_segment.h"
#include "numeric_utils.h"
#include "geom_utils.h"
#include "vpt_utils.h"
#include "camera.h"
#include "vanishing_point_model.h"

#include "drawing.h"

#include "vector_utils.tpp"
#include "format_utils.tpp"
#include "numerical_jacobian.tpp"

using namespace indoor_context;
using namespace toon;

// Variance of the reprojection error, in pixels.
// Recall that in the normal distribution the variance is sigma *squared*
static const double kLineSigmaSqr = 6.;

// Num segments to sample along each axis
const int kNumSegmentsPerAxis = 10;


int main(int argc, char **argv) {
	if (argc != 3) {
		cout << "Usage: synthetic_vpt_experiment DISTANCE OUTPUT.CSV\n";
		return -1;
	}
	double distance = atof(argv[1]);
	string outfile = argv[2];

	int nx = 200, ny = 200;
	Vec2I size = makeVector(nx, ny);
	Vec3 zero = Zeros;

	// Camera params
	Mat3 H = Identity;
	H[0] = makeVector(nx/2., 0, nx/2.);
	H[1] = makeVector(0, ny/2., ny/2.);

	// Construct camera
	LinearCamera intrinsics(H, size);
	SE3<> identity_pose;
	PosedImage image(identity_pose, &intrinsics);
	image.Allocate();
	image.rgb.Clear(Colors::white());

	// Sample data
	SO3<> Rw_true;
	vector<LineSeg> segments;
	vector<int> axes;
	SampleRotationAndLines(kNumSegmentsPerAxis, kLineSigmaSqr,
												 image.pc(), image.size(), 
												 Rw_true, segments, axes);

	CHECK_EQ(segments.size(), axes.size());
	vector<pair<LineObservation,int> > data;
	for (int i = 0; i < segments.size(); i++) {
		data.push_back(make_pair(LineObservation(&image.pc(), segments[i]), axes[i]));
	}

	// Create the optimizer
	VanishingPointModel model(kLineSigmaSqr);
	RotationOptimizer optimizer(model, data);
	//optimizer.verify_gradients = true;

	// Do gradient descent
	SO3<> Rw_init = Rw_true * SO3<>::exp(makeVector(distance, distance, distance));
	SO3<> Rw_basin = optimizer.Compute(Rw_init, data);

	// Write csv
	ofstream out(outfile.c_str());
	out << "\"Iteration\",\"Log likelihood\",\"Error wrt Ground Truth\"\n";
	boost::format line("%d,%f,%f\n");
	for (int i = 0; i <= optimizer.num_steps; i++) {
		out << line % i % optimizer.fs[i] % RotationDistance(optimizer.Rs[i], Rw_true);
	}	

	// Report
	DREPORT(optimizer.num_steps);
	DREPORT(Rw_true, Rw_basin);
	DREPORT(RotationDistance(Rw_true, Rw_init));
	DREPORT(model.ComputeCompleteLogLik(data, Rw_true));
	DREPORT(model.ComputeCompleteLogLik(data, Rw_init));
	DREPORT(model.ComputeCompleteLogLik(data, Rw_basin));
	DREPORT(RotationDistance(Rw_true, Rw_basin));

	// Vizualize
	OutputVptViz(image, segments, axes, Rw_true, "out/Rw_true.pdf");
	OutputVptViz(image, segments, axes, Rw_init, "out/Rw_init.pdf");
	OutputVptViz(image, segments, axes, Rw_basin, "out/Rw_basin.pdf");

	return 0;
}
