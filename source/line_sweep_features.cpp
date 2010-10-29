#include "line_sweep_features.h"

#include <boost/foreach.hpp>

#include "canvas.h"
#include "timer.h"
#include "image_utils.h"

#include "vector_utils.tpp"

namespace indoor_context {
	namespace {
		lazyvar<float> gvWallPenalty("ManhattanDP.DefaultWallPenalty");
		lazyvar<float> gvOcclusionPenalty("ManhattanDP.DefaultOcclusionPenalty");
	}

	void LineSweepDPScore::OutputOrientViz(const string& path) {
		ImageRGB<byte> orient_canvas;
		ImageCopy(input_image->rgb, orient_canvas);
		line_sweeper.DrawOrientViz(orient_canvas);
		line_detector.DrawSegments(orient_canvas);
		WriteImage(path, orient_canvas);
	}

	void LineSweepDPScore::OutputLineViz(const string& path) {
		FileCanvas canvas(path, asToon(input_image->sz()));
		canvas.DrawImage(input_image->rgb);
		canvas.SetLineWidth(3.0);
		for (int i = 0; i < 3; i++) {
			BOOST_FOREACH(const LineDetection& det, line_detector.detections[i]) {
				canvas.StrokeLine(det.seg, Colors::primary(i));
			}
		}
	}



	void LineSweepDPScore::Compute(const PosedImage& image) {
		input_image = &image;

		// Detect lines and guess initial orientation
		TIMED("Detect lines")
			line_detector.Compute(image);
		TIMED("Estimate orientations")
			line_sweeper.Compute(image, line_detector.detections);

		// Convert the line sweeper labels to a score matrix
		objective.Resize(image.nx(), image.ny());
		for (int y = 0; y < image.ny(); y++) {
			const int* inrow = line_sweeper.orient_map[y];
			for (int i = 0; i < 3; i++) {
				float* outrow = objective.pixel_scores[i][y];
				for (int x = 0; x < image.nx(); x++) {
					outrow[x] = (inrow[x] == i ? 1.0 : 0.0);
				}
			}
		}
		objective.wall_penalty = *gvWallPenalty;
		objective.occl_penalty = *gvOcclusionPenalty;
	}





	void LineSweepFeatureGenerator::Compute(const PosedImage& image) {
		input_image = &image;

		// Compute line sweeps
		line_detector.Compute(image);
		line_sweeper.Compute(image, line_detector.detections);

		// We must never overwrite old features if !features.unique()
		// because that means another object is holding a reference to the
		// old features and may wish to still use them.
		int n = image.nx()*image.ny();
		if (feature_count != n || !features.unique()) {
			feature_count = n;
			features.reset(new FeatureVec[n]);
		}

		// Generate the features
		int i = 0;
		for (int y = 0; y < image.ny(); y++) {
			const PixelRGB<byte>* imrow = image.rgb[y];
			const int* sweeprow = line_sweeper.orient_map[y];
			for (int x = 0; x < image.nx(); x++) {
				FeatureVec& ftr = features[i++];
				ftr[0] = x;
				ftr[1] = y;
				ftr[2] = imrow[x].r;
				ftr[3] = imrow[x].g;
				ftr[4] = imrow[x].b;
				ftr[5] = sweeprow[x] == 0 ? 1.0 : 0.0;
				ftr[6] = sweeprow[x] == 1 ? 1.0 : 0.0;
				ftr[7] = sweeprow[x] == 2 ? 1.0 : 0.0;
			}
		}
	}
}
