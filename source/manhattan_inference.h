#pragma once

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/ptr_container/ptr_map.hpp>

#include <mex.h>

#include "common_types.h"
#include "map.h"
#include "manhattan_dp.h"
#include "camera.h"
#include "bld_helpers.h"
#include "line_sweep_features.h"

namespace indoor_context {
	// Solves the inference problems involved in training the manhattan
	// DP reconstructor
	class ManhattanInference {
	public:
		static const int kFeatureLength = LineSweepFeatureGenerator::kFeatureLength;
		typedef LineSweepFeatureGenerator::FeatureVec FeatureVec;
		typedef toon::Vector<kFeatureLength*3> PsiVec;

		const PosedImage* input_image;
		Mat3 input_floorToCeil;
		shared_array<FeatureVec> input_features;

		// Ground truth labels
		MatI gt_labels;

		// Features for each pixel
		LineSweepFeatureGenerator feature_gen;

		// Reconstruction
		DPObjective score_func;
		ManhattanDPReconstructor reconstructor;

		// Default constructor
		ManhattanInference() : input_image(NULL) { }

		// Compute features and ground truth
		void Prepare(const PosedImage& image,
								 const proto::FloorPlan& floorplan,
								 shared_array<FeatureVec> features);

		// Compute features
		void Prepare(const PosedImage& image,
								 const Mat3& floorToCeil,
								 shared_array<FeatureVec> features);

		// Compute ground truth labels
		void ComputeGroundTruth(const proto::FloorPlan& floorplan);

		// Compute features, build a score function, and run the DP inference
		void ComputeReconstruction(const toon::Vector<>& weights);

		// Compute features, build a score function, and run the DP
		// inference. This version modifies the score function to hunt for
		// the most violated constraint wrt the ground truth for this
		// frame. If ref_labels is not supplied then this->gt_labels is
		// used.
		void ComputeMostViolated(const toon::Vector<>& weights);
		void ComputeMostViolated(const toon::Vector<>& weights,
														 const MatI& ref_labels);

		// Compute a vector X such that for any W, the inner product <X,W>
		// equals the value of the score function for the given
		// reconstruction under the weight vector W.
		void ComputePsi(const MatI& labels, PsiVec& psi);

		// Do the prediction given the specified weight vector If
		// ref_labels is not null then the score function will be
		// configured so that the DP seeks the worst solution given the
		// current weight vector.
		void ComputeScoreFunc(const toon::Vector<>& weights,
													const MatI* ref_labels);
	};

	class FrameStore {
	public:
		// The cache of maps: sequence -> map
		boost::ptr_map<string, Map> maps;
		// The cache of floorplans: sequence -> map
		boost::ptr_map<string, proto::TruthedMap> gt_maps;
		// The cache of frames: caseName -> frame
		boost::ptr_map<string, ManhattanInference> cases;

		// Get the singleton isntance of this class
		static FrameStore& instance();
		// Parse a case name of the form "<sequenceName>:<frameIndex>"
		static pair<string,int> ParseCaseName(const string& case_name);

		// Get the instance for the given case name (format is
		// "<sequence>:<frameIndex>", eg "foo_dataset:25")
		ManhattanInference& Get(const string& case_name);
		// Get the instance for the given case name (format is
		// "<sequence>:<frameIndex>", eg "foo_dataset:25")
		ManhattanInference& GetOrInit(const string& case_name);
		// Get the map for the given sequence name (e.g. "lab_kitchen1")
		Map& GetMap(const string& sequence_name);
		// Get the map for the given sequence name (e.g. "lab_kitchen1")
		const proto::FloorPlan& GetFloorPlan(const string& sequence_name);

		// If the specified map is not already loaded then load it
		void EnsureMapLoaded(const string& sequence_name);

		// Clear the cache of frame data
		void Clear();
	};

}
