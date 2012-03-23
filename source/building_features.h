#pragma once

#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "filters.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"

#include "integral_image.tpp"

namespace indoor_context {
	class PosedImage;

	// Accumulates features in a local neighbourhood and for neighbours
	class AccumulatedFeatures {
	public:
		Vec2I size;  // size of input matrix
		IntegralImage<float> integ;
		boost::ptr_vector<MatF> results;
		void Prepare(const MatF& input);
		void Compute(int radius, bool include_nbrs=true);
		void Compute(const MatF& input, int radius, bool include_nbrs=true);
	};

	// Generates a range of features
	class PhotometricFeatures {
	public:
		const PosedImage* input;

		// The enabled feature components (e.g. "hsv", "gabor")
		set<string> components;
		// The features generated on last call to Compute()
		vector<const MatF*> features;
		// A textual explanation for each matrix in the above
		vector<string> feature_strings;

		// RGB and HSV features
		boost::ptr_vector<MatF> rgb_features;
		boost::ptr_vector<MatF> hsv_features;

		// Gabor features
		GaborFilters gabor;
		boost::ptr_vector<MatF> gabor_features;

		// Line sweeps
		GuidedLineDetector line_detector;
		IsctGeomLabeller line_sweeper;
		boost::ptr_vector<MatF> sweep_features;
		// accum_sweeps
		AccumulatedFeatures accum[3];

		// Ground truth
		boost::ptr_vector<MatF> gt_features;

		// Initialize empty
		PhotometricFeatures();
		// Initialize with the given feature set (see Configure() below)
		PhotometricFeatures(const string& config);

		// Configure the features with a string describing the feature set
		// Example feature set:
		//   "rgb,hsv,sweeps,accum_sweeps"
		//   "all,-rgb,-hsv"   -- all except RGB and HSV
		//   "all,gt"          -- all including ground truth orientations
		//   "default"         -- use the feature spec in the relevant gvar
		void Configure(const string& config);
		// Return true if the given component was active in the config string
		bool IsActive(const string& component);

		// Compute features. If ground truth is NULL then the feature set
		// must not include "gt".
		void Compute(const PosedImage& image);
		void Compute(const PosedImage& image, const MatI* gt_orients);
	};
}
