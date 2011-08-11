#include "gaussian_pyramid.h"

#include "filters.h"
#include "timer.h"

#include "numeric_utils.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	GaussianPyramid::GaussianPyramid() {
		GaussFunction g(1.0);
		lowpass.reset(g.MakeSeperatedFilter());
	}

	// Empty destructor for scoped_ptr (ugh)
	GaussianPyramid::~GaussianPyramid() {
	}

	void GaussianPyramid::ComputeLevels(const MatF& input, int n) {
		levels.resize(n);
		buffers.resize(n-1);

		if (levels[0] == NULL) {
			levels[0].reset(new MatF);
		}
		*levels[0] = input;  // Copy entire matrix

		for (int i = 1; i < n; i++) {
			Vec2I size = matrix_size(*levels[i-1]);
			buffers[i-1].Resize(size[1], size[0]);
			lowpass->RunSequential(*levels[i-1], buffers[i-1]);
			if (levels[i] == NULL) {
				levels[i].reset(new MatF(size[1]/2, size[0]/2));
			} else {
				levels[i]->Resize(size[1]/2, size[0]/2);  // will only re-allocate if necessary
			}
			Downsample(buffers[i-1], 2, *levels[i]);
		}
	}

	void GaussianPyramid::ComputeCube(const MatF& input, int n) {
		int maxs = 1<<(n-1);
		CHECK(input.Rows()%maxs == 0 && input.Cols()%maxs == 0)
			<< "Input array size must be divisible by 2^(n-1), size was: " << matrix_size(input);

		// Compute the pyramid
		ComputeLevels(input, n);

		// Upsample each level
		slices.resize(n);
		slices[0] = *levels[0];  // yes, copy again
		for (int i = 1; i < n; i++) {
			int scale = 1 << i;
			slices[i].Resize(input.Rows(), input.Cols());
			CHECK_EQ(matrix_size(*levels[i])*scale, matrix_size(input))
				<< "i="<<i<<", size="<<matrix_size(*levels[i])<<", scale="<<scale;
			Upsample(*levels[i], scale, slices[i]);
		}
	}

	void GaussianPyramid::ReleaseBuffers() {
		buffers.clear();
	}



	void PyramidFeatureGen::Compute(const MatF& input, int num_scales, double nbr_dist) {
		TIMED("Generate pyramid") pyramid.ComputeCube(input, num_scales);
		
		features.clear();
		nbr_features.resize(num_scales*4);
		int k = 0;

		TIMED("Generate translations") 
		for (int i = 0; i < num_scales; i++) {
			int scale = nbr_dist * (1<<i);

			// No translation
			features.push_back(&pyramid.slices[i]);

			for (int t = -1; t <= 1; t+=2) {
				// Horizontal translation +/- T
				nbr_features[k].Resize(input.Rows(), input.Cols());
				features.push_back(&nbr_features[k]);
				ShiftHoriz(pyramid.slices[i], nbr_features[k], t*scale);
				k++;

				// Vertical translation +/- T
				nbr_features[k].Resize(input.Rows(), input.Cols());
				features.push_back(&nbr_features[k]);
				ShiftVert(pyramid.slices[i], nbr_features[k], t*scale);
				k++;
			}
		}
	}
}
