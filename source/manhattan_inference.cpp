#include <mex.h>

#include "manhattan_inference.h"

#include "common_types.h"
#include "manhattan_dp.h"
#include "camera.h"
#include "timer.h"
#include "clipping.h"
#include "bld_helpers.h"
#include "safe_stream.h"

#include "io_utils.tpp"
#include "table.tpp"

namespace indoor_context {
	using namespace toon;

	void ManhattanInference::Compute(const PosedImage& image,
	                                 const Mat3& floor2ceil,
	                                 const toon::Vector<>& weights) {
		input_image = &image;
		input_floor2ceil = floor2ceil;

		line_detector.Compute(image);
		line_sweeper.Compute(image, line_detector.detections);

		ComputeScoreFunc(weights);
		ComputeReconstruction();
	}

	void ManhattanInference::ComputeScoreFunc(const toon::Vector<>& w) {
		// Compute the score function
		scorefunc.Resize(input_image->nx(), input_image->ny());
		Vector<kFeatureLength> classifiers[] = {
				w.slice(0, kFeatureLength),
				w.slice(kFeatureLength, kFeatureLength),
				w.slice(kFeatureLength*2, kFeatureLength)
		};
		Vector<kFeatureLength> ftr;
		for (int y = 0; y < input_image->ny(); y++) {
			const PixelRGB<byte>* imrow = input_image->rgb[y];
			const int* sweeprow = line_sweeper.orient_map[y];
			float* scorerows[] = {
					scorefunc.pixel_scores[0][y],
					scorefunc.pixel_scores[1][y],
					scorefunc.pixel_scores[2][y]
			};
			for (int x = 0; x < input_image->nx(); x++) {
				ftr[0] = imrow[x].r;
				ftr[1] = imrow[x].g;
				ftr[2] = imrow[x].b;
				ftr[3] = sweeprow[x] == 0 ? 1.0 : 0.0;
				ftr[4] = sweeprow[x] == 1 ? 1.0 : 0.0;
				ftr[5] = sweeprow[x] == 2 ? 1.0 : 0.0;
				for (int i = 0; i < 3; i++) {
					scorerows[i][x] = classifiers[i] * ftr;
				}
			}
		}
	}

	void ManhattanInference::ComputeReconstruction() {
		reconstructor.Compute(*input_image, input_floor2ceil, scorefunc);
	}
}
