#include <stdlib.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

#include <TooN/so3.h>

#include "common_types.h"
#include "canvas.h"
#include "colors.h"
#include "clipping.h"
#include "line_segment.h"
#include "numeric_utils.h"
#include "geom_utils.h"
#include "vpt_utils.h"
#include "camera.h"
#include "vanishing_point_model.h"

#include "canvas.h"
#include "drawing.h"

#include "vector_utils.tpp"
#include "format_utils.tpp"
#include "numerical_jacobian.tpp"

using namespace indoor_context;
using namespace toon;

// Variance of the reprojection error, in pixels.
// Recall that in the normal distribution the variance is sigma *squared*
static const double kLineSigmaSqr = 6.;







class LineSegmentSampler {
public:
	boost::mt19937 rng;
	boost::normal_distribution<> noise_model;
	boost::variate_generator<boost::mt19937,
													 boost::normal_distribution<> > noise;

	LineSegmentSampler(double sigma) : noise_model(0, sigma),
																		 noise(rng, noise_model) {
	}

	LineSeg Sample(const Vec3& vpt, const Vec2I& size);
};

LineSeg LineSegmentSampler::Sample(const Vec3& vpt, const Vec2I& size) {
	Vec2 xa, xb, ya, yb;
	xa[0] = 1. * size[0] * rand() / RAND_MAX;
	xa[1] = 1. * size[1] * rand() / RAND_MAX;
	Vec3 line = unproject(xa) ^ vpt;
	ClipLineToImage(line, asIR(size), xa, xb);

	double ta = 1. * rand() / RAND_MAX;
	double tb = 1. * rand() / RAND_MAX;
	ya = ta*xa + (1.-ta)*xb;
	yb = tb*xa + (1.-tb)*xb;

	// Add noise
	ya += makeVector(noise(), noise());
	yb += makeVector(noise(), noise());

	// Return
	return LineSeg(unproject(ya), unproject(yb));	
}




int main(int argc, char **argv) {
	const int kNumSegmentsPerAxis = 5;

	int nx = 200, ny = 200;
	Vec2I size = makeVector(nx, ny);
	Vec3 zero = Zeros;

	// Camera params
	Mat3 H = Identity;
	H[0] = makeVector(nx/2., 0, nx/2.);
	H[1] = makeVector(0, ny/2., ny/2.);
	LinearCamera intrinsics(H, size);
	SE3<> identity_pose;
	PosedImage image(identity_pose, &intrinsics);
	image.Allocate();
	image.rgb.Clear(Colors::white());

	// Sample true rotation
	Vec3 m_true = RandomVector<3>();
	SO3<> Rw_true = SO3<>::exp(m_true);
	Vec3 m_init = m_true + makeVector(.1, .1, .1);
	SO3<> Rw_init = SO3<>::exp(m_init);

	// Sample line segments
	LineSegmentSampler seg_sampler(kLineSigmaSqr);
	vector<LineSeg> segments[3];
	vector<LineSeg> all_segments;
	vector<int> all_axes;
	vector<pair<LineObservation,int> > observations;
	for (int i = 0; i < 3; i++) {
		PixelRGB<byte> color = Colors::primary(i);
		Vec3 vpt = ProjectVanishingPoint(i, Rw_true, image.pc());

		// Sample lines
		for (int j = 0; j < kNumSegmentsPerAxis; j++) {
			LineSeg seg = seg_sampler.Sample(vpt, image.size());
			segments[i].push_back(seg);
			all_segments.push_back(seg);
			all_axes.push_back(i);
			observations.push_back(make_pair(LineObservation(&image.pc(), seg), i));
		}
	}

	// Pick a spot to start initialization

	// Set up function
	/*for (int i = 0; i < 3; i++) {
		int axis = i;
		Vec3 vpt = ProjectVanishingPoint(axis, Rw_init, image.pc());

		BOOST_FOREACH(const LineSeg& seg, segments[i]) {
			TITLE("Segment " << seg.midpoint() << " [axis=" << axis << "]");
			boost::function<double(const Vec3&)> ff_dist =
				boost::bind(&SignedDist,
										seg,
										axis,
										ref(image.pc()),
										ref(Rw_init),
										_1);

			boost::function<double(const Vec3&)> ff_loglik =
				boost::bind(&ComputeSegmentLogLik,
										seg,
										axis,
										ref(image.pc()),
										ref(Rw_init),
										_1);

			boost::function<double(const Vec3&)> ff_vpt = 
				boost::bind(&SignedReprojError, ref(seg), _1);

			boost::function<double(const Vec3&)> ff_reproj = 
				boost::bind(&SignedPointLineDist, seg.start, _1);

			//Vec3 grad = ComputeDistGradient(seg, axis, image.pc(), Rw_init);
			//Vec3 gt_grad = NumericJacobian(ff_dist, zero, 1e-6);

			Vec3 grad = ComputeLogLikGradient(seg, axis, image.pc(), Rw_init);
			Vec3 gt_grad = NumericJacobian(ff_loglik, zero, 1e-6);

			DLOG << "Analytic gradient: " << grad;
			DLOG << "True gradient: " << gt_grad;
			double err = norm(grad-gt_grad);
			if (err > 1e-5) {
				DLOG << "  ** INCORRECT **";
			}
		}
		}*/

	/*boost::function<double(const Vec3&)> ff_compl = 
		boost::bind(&ComputeCompleteLogLik, ref(observations), Rw_init, _1);

	Vec3 grad = ComputeCompleteLogLikGradient(observations, Rw_init);
	Vec3 gt_grad = NumericJacobian(ff_compl, zero, 1e-6);

	DLOG << "Analytic gradient: " << grad;
	DLOG << "True gradient: " << gt_grad;
	double err = norm(grad-gt_grad);
	if (err > 1e-5) {
		DLOG << "  ** INCORRECT **";
	}

	return 0;*/



	/*TITLED("Moving away from truth")
	for (int i = 0; i < 20; i++) {
		double dist = i / 5.;
		Vec3 mprime = m + makeVector(dist, dist, dist);
		SO3<> Rw_prime = SO3<>::exp(mprime);
		double loglik = ComputeCompleteLogLik(observations, Rw_prime);
		DLOG << "At dist="<<dist<<", log likelihood is "<<loglik;

		INDENTED for (int i = 0; i < observations.size(); i++) {
			Vec3 vpt = ProjectVanishingPoint(observations[i].second, Rw_prime, image.pc());
			const LineSeg& seg = observations[i].first.segment;
			double dist = ComputeReprojError(seg, vpt);
			//DLOG << "Reproj dist for seg " << i << " = " << dist;
		}
		
		string path = fmt("out/Rw_dist_%02d.png", i);
		OutputVptViz(image, allsegs, Rw_prime, path);
		}*/

	VanishingPointModel model(kLineSigmaSqr);
	RotationOptimizer optimizer(model, observations);

	SO3<> Rw_basin = optimizer.Compute(Rw_init, observations);
	for (int i = 0; i <= optimizer.num_steps; i++) {
		TITLED("Iteration "<<i) {
			DLOG << "f=" << optimizer.fs[i];
		}
	}

	DREPORT(model.ComputeCompleteLogLik(observations, Rw_true));
	DREPORT(model.ComputeCompleteLogLik(observations, Rw_init));
	DREPORT(Rw_true, Rw_basin);

	// Draw line segments
	OutputVptViz(image, all_segments, all_axes, Rw_true, "out/Rw_true.pdf");
	OutputVptViz(image, all_segments, all_axes, Rw_init, "out/Rw_init.pdf");
	OutputVptViz(image, all_segments, all_axes, Rw_basin, "out/Rw_basin.pdf");

	return 0;
}
