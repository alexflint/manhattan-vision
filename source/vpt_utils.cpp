#include "vpt_utils.h"

#include <TooN/so3.h>

#include "common_types.h"
#include "line_segment.h"
#include "camera.h"
#include "colors.h"
#include "clipping.h"
#include "geom_utils.h"

#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	Vec3 ProjectVanishingPoint(int axis,
														 const SO3<>& R,
														 const PosedCamera& camera) {
		return camera.RetToIm(camera.pose().get_rotation() * R * GetAxis<3>(axis));
	}

	double ComputeReprojError(const LineSegment& segment,
														const Vec3& vpt) {
		return EuclPointLineDist(segment.start, vpt^segment.midpoint());
	}

	double SignedReprojError(const LineSegment& segment,
													 const Vec3& vpt) {
		return SignedPointLineDist(segment.start, vpt^segment.midpoint());
	}

	int ComputeMostLikelyAxis(const PosedCamera& cam,
														const SO3<>& R,
														const LineSegment& segment) {
		double minerr = INFINITY;
		int axis = -1;
		for (int i = 0; i < 3; i++) {
			Vec3 vpt = ProjectVanishingPoint(i, R, cam);
			double err = ComputeReprojError(segment, vpt);
			if (err < minerr) {
				axis = i;
				minerr = err;
			}
		}
		return axis;
	}

	void SampleRotationAndLines(int nlines,
															double line_sigma_sqr,
															const PosedCamera& camera,
															const Vec2I& image_size,
															SO3<>& R,
															vector<LineSegment>& segments,
															vector<int>& axes) {
		// Sample true rotation
		R = SO3<>::exp(RandomVector<3>());

		// Sample line segments
		LineSegmentSampler seg_sampler(line_sigma_sqr);
		for (int i = 0; i < 3; i++) {
			Vec3 vpt = ProjectVanishingPoint(i, R, camera);
			for (int j = 0; j < nlines; j++) {
				LineSegment seg = seg_sampler.Sample(vpt, image_size);
				segments.push_back(seg);
				axes.push_back(i);
			}
		}
	}

	LineSegment LineSegmentSampler::Sample(const Vec3& vpt, const Vec2I& size) {
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
		return LineSegment(unproject(ya), unproject(yb));	
	}
}
