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

// The prior probability of a spurious detection. 
static const double kSpuriousPrior = .2;

// The probability of a specific line detection given that it is a
// spurious detection. Technically, this should equal 1/(num possible
// line detections) since all spurious detections are equally probable
// and they should sum to 1.
static const double kSpuriousLik = 1e-6;

// Num segments to sample along each axis
const int kNumSegmentsPerAxis = 12;


int main(int argc, char **argv) {
	if (argc != 1) {
		cout << "Usage: em_vpt_experiment DISTANCE OUTPUT.CSV\n";
		return -1;
	}
	//double distance = atof(argv[1]);
	//string outfile = argv[2];

	int nx = 200, ny = 200;
	Vec2I size = makeVector(nx, ny);
	Vec3 zero = Zeros;

	// Camera params
	Mat3 H = Identity;
	H[0] = makeVector(nx, 0., 0.);
	H[1] = makeVector(0., ny, 0.);

	SE3<> identity_pose;
	SE3<> rand_pose(SO3<>::exp(RandomVector<3>()), RandomVector<3>());
	DREPORT(rand_pose);

	// Construct camera
	LinearCamera intrinsics(H, size);
	PosedImage image(rand_pose, &intrinsics);
	image.Allocate();
	image.rgb.Clear(Colors::white());

	// Sample data
	SO3<> Rw_true;
	vector<LineSeg> segments;
	vector<int> axes;
	SampleRotationAndLines(kNumSegmentsPerAxis, 2.,
												 image.pc(), image.size(), Rw_true, 
												 segments, axes);

	CHECK_EQ(segments.size(), axes.size());
	vector<pair<LineObservation,int> > data;
	vector<LineObservation> observations;
	for (int i = 0; i < segments.size(); i++) {
		LineObservation obs(&image.pc(), segments[i]);
		observations.push_back(obs);
		data.push_back(make_pair(obs, axes[i]));
	}
	
	// Pick a start point
	double d = -.2;
	SO3<> Rw_init = Rw_true * SO3<>::exp(makeVector(d, d, d));

	// Create the model
	VanishingPointModel model(6.);

	// Check gradients
	MatD rand_resps(segments.size(), 4);
	MatD perfect_resps(segments.size(), 4);
	for (int i = 0; i < segments.size(); i++) {
		for (int j = 0; j < 4; j++) {
			rand_resps[i][j] = 1. * rand() / RAND_MAX;
			perfect_resps[i][j] = data[i].second == j ? 1. : 0.;
		}
	}
	Vec3 true_G = model.ComputeExpectedLogLik_NumericGradient(observations,
																														perfect_resps, Rw_init, 1e-8);
	Vec3 ana_G = model.ComputeExpectedLogLikGradient(observations,
																									 perfect_resps, Rw_init);
	DLOG << "Analytic gradient: " << ana_G;
	DLOG << "True gradient: " << true_G;
	//CHECK_LT(norm(true_G - ana_G), 1e-5);

	VanishingPointEstimator estimator(observations, model);
	SO3<> Rw_basin = estimator.Compute(Rw_init);

	Vec3 true_G_basin = model.ComputeExpectedLogLik_NumericGradient(observations,
																														perfect_resps, Rw_basin, 1e-8);
	Vec3 ana_G_basin = model.ComputeExpectedLogLikGradient(observations,
																									 perfect_resps, Rw_basin);

	DREPORT(true_G_basin);
	DREPORT(ana_G_basin);
	DREPORT(norm(ana_G_basin - true_G_basin));

	DREPORT(RotationDistance(Rw_init, Rw_true));
	DREPORT(RotationDistance(Rw_basin, Rw_true));

	for (int i = 0; i <= 10; i++) {
		OutputVptViz(image, segments, axes, estimator.optimizer.Rs[i],
								 fmt("out/Rw_iter%03d.pdf", i));
	}
		

	// Vizualize
	OutputVptViz(image, segments, axes, Rw_true, "out/Rw_true.pdf");
	OutputVptViz(image, segments, axes, Rw_init, "out/Rw_init.pdf");
	OutputVptViz(image, segments, axes, Rw_basin, "out/Rw_basin.pdf");

	return 0;
}
