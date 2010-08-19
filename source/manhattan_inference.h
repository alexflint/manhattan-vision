#pragma once

#include <mex.h>

#include "common_types.h"
#include "manhattan_dp.h"
#include "camera.h"

#include "table.tpp"

namespace indoor_context {
	// Solves the inference problems involved in training the manhattan DP reconstructor
	class ManhattanInference {
	public:
		static const int kFeatureLength = 6;

		const PosedImage* input_image;
		Mat3 input_floor2ceil;

		GuidedLineDetector line_detector;
		IsctGeomLabeller line_sweeper;

		ManhattanDP::ScoreFunction scorefunc;
		ManhattanDPReconstructor reconstructor;

		// Compute features and a score function
		void Compute(const PosedImage& pim,
		             const Mat3& floor2ceil,
		             const toon::Vector<>& weights);

		// Do the prediction given the specified weight vector
		void ComputeScoreFunc(const toon::Vector<>& w);
		// Compute the reconstruction given current score function
		void ComputeReconstruction();
	};
}
