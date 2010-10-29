#include "common_types.h"
#include "camera.h"
#include "guided_line_detector.h"
#include "manhattan_dp.h"

namespace indoor_context {
	// Compute costs by sweeping lines
	class LineSweepDPScore {
	public:
		const PosedImage* input_image;
		GuidedLineDetector line_detector;
		IsctGeomLabeller line_sweeper;
		DPObjective objective;

		// Initialize empty
		LineSweepDPScore() { }
		// Initialize and compute
		LineSweepDPScore(const PosedImage& image) {
			Compute(image);
		}

		// Compute a DP score function by sweeping lines in the specified image
		void Compute(const PosedImage& image);

		// Draw lines and line sweeps
		void OutputOrientViz(const string& path);
		// Draw the line detections
		void OutputLineViz(const string& path);
	};

	// Generates features for each pixel
	class LineSweepFeatureGenerator {
	public:
		static const int kFeatureLength = 3;
		// FeatureVec constitutes most of the heap usage, so make it floats	
		typedef toon::Vector<kFeatureLength,float> FeatureVec;

		// The input image
		const PosedImage* input_image;
		// Input image dimensions
		Vec2I dims;

		// Line detector and sweeper
		GuidedLineDetector line_detector;
		IsctGeomLabeller line_sweeper;

		// Linearized per-pixel features
		// We use a scoped_array here because this array consumes most of
		// the memory in dp_load_cases
		shared_array<FeatureVec> features;
		// The length of the features array
		int feature_count;

		// Initialize empty
		LineSweepFeatureGenerator() : feature_count(0), input_image(NULL) { }
		// Compute features for each pixel
		void Compute(const PosedImage& pim);
	};
}
