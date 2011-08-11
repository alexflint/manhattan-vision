#pragma once

#include "common_types.h"

#include <boost/ptr_container/ptr_vector.hpp>

namespace indoor_context {
	class SeperatedFilter;

	// Represents a gaussian pyramid
	class GaussianPyramid {
	public:
		// The Gaussian filter
		scoped_ptr<SeperatedFilter> lowpass;
		// The levels of the pyramid
		vector<boost::shared_ptr<MatF> > levels;
		// The smoothed versions of each pyramid level. This vector has
		// one fewer elements than the one above since the last level
		// doesn't need to be smoothed.
		boost::ptr_vector<MatF> buffers;
		// Each level of the pyramid upsampled to the full size. Only
		// computed when ComputeCube() is called.
		boost::ptr_vector<MatF> slices;

		// Empty constructor and destructor for scoped_ptr (ugh)
		GaussianPyramid();
		~GaussianPyramid();

		// Compute N levels of the pyramid
		void ComputeLevels(const MatF& input, int n);
		// Compute the levels, then upsample them each to the size of the original frame.
		void ComputeCube(const MatF& input, int n);
		// Clear buffers. This decreases memory consumption by 50% but
		// increases time taken by Compute() as it will have to
		// re-allocate buffers.
		void ReleaseBuffers();
	};

	// Generates features using a gaussian pyramid
	class PyramidFeatureGen {
	public:
		// Pointers to all matrices (translated and non-translated)
		vector<MatF*> features;
		// The gaussian pyramid used for filtering
		GaussianPyramid pyramid;
		// The translated features
		boost::ptr_vector<MatF> nbr_features;

		// Compute features. Second parameters is the number of gaussian
		// pyramid levels to compute, the second is the number of pixels
		// to compute neighbours at each level (scales with the pyramid)
		void Compute(const MatF& input, int num_scales, double nbr_dist);
	};
}
