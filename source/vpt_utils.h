#pragma once

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

#include <TooN/so3.h>

#include "common_types.h"

namespace indoor_context {
	class LineSegment;
	class PosedCamera;

	// Compute the location of a vanishing point in an image given a
	// rotation of the world and a calibrated camera.
	Vec3 ProjectVanishingPoint(int axis,
														 const toon::SO3<>& R,
														 const PosedCamera& cam);

	// Compute reprojection error in pixels
	double ComputeReprojError(const LineSegment& segment,
														const Vec3& vpt);

	// Signed version of above (see SignedPointLineDist in geom_utils.h)
	double SignedReprojError(const LineSegment& segment,
													 const Vec3& vpt);

	// Identify the max likelihood axis for a line segment
	int ComputeMostLikelyAxis(const PosedCamera& cam,
														const toon::SO3<>& R,
														const LineSegment& segment);

	// Sample a rotation matrix and vpt-aligned line segments
	void SampleRotationAndLines(int num_lines_per_axis,
															double line_sigma_sqr,
															const PosedCamera& camera,
															const Vec2I& image_size,
															toon::SO3<>& R,
															vector<LineSegment>& segments,
															vector<int>& axes);

	class LineSegmentSampler {
	public:
		boost::mt19937 rng;
		boost::normal_distribution<> noise_model;
		boost::variate_generator<boost::mt19937,
														 boost::normal_distribution<> > noise;

		LineSegmentSampler(double sigma) : noise_model(0, sigma),
																			 noise(rng, noise_model) {
		}

		LineSegment Sample(const Vec3& vpt, const Vec2I& size);
	};
}
