#pragma once

#include <boost/ptr_container/ptr_vector.hpp>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"

#include "payoff_helpers.h"
#include "manhattan_dp.h"
#include "manhattan_ground_truth.h"

#include "stereo_payoffs.h"
#include "point_cloud_payoffs.h"
#include "line_sweep_features.h"
#include "monocular_payoffs.h"
#include "building_features.h"

namespace indoor_context {
	class ManhattanHyperParameters;
	class TrainingInstance;
	class ManhattanHypothesis;
	class FeatureManager;

	void PredictGridLabels(const ManhattanHypothesis& hyp,
												 const DPGeometry& geometry,
												 MatI& image_orientations);

	void PredictDepth(const ManhattanHypothesis& hyp,
										const DPGeometry& geometry,
										MatF& image_depths);

	// Common representation for ground truth instances and DP solutions
	class ManhattanHypothesis {
	public:
		ManhattanHypothesis();
		ManhattanHypothesis(const VecI& path_ys,
												const VecI& path_axes,
												int num_corners,
												int num_occlusions);
		VecI path_ys;
		VecI path_axes;
		int num_corners;
		int num_occlusions;
		TrainingInstance* instance;   // Hack because loss() doesn't pass the instance to us
	};

	class TrainingInstance {
	public:
		string sequence;

		const Frame* frame;
		DPGeometryWithScale geometry;
		//PayoffFeatures features;
		ManhattanGroundTruth gt;
		ManhattanHypothesis gt_hypothesis;
		MatF loss_terms;

		// Configure with data and ground truth
		void Configure(const Frame& frame, const proto::FloorPlan& gt);

		// Configure various loss functions
		void ConfigureL1Loss(double kcondition=0.);
		void ConfigureLabellingLoss(double kcondition=0.);
		void ConfigureDepthLoss(double kcondition=0.);

		// Compute pixel--wise labels (in grid coords) for a hypothesis
		MatI ComputeLabels(const ManhattanHypothesis& hyp) const;
		// Compute pixel--wise depths (in grid coords) for a hypothesis
		MatF ComputeDepths(const ManhattanHypothesis& hyp) const;

		// Get ground truth data
		ManhattanHypothesis GetGroundTruth() const;  // TODO: return reference
		MatI GetGroundTruthLabels() const;
		MatD GetGroundTruthDepths() const;
		MatF GetLossTerms() const;

		// Draw ground truth as labels
		void OutputGroundTruthViz(const string& path) const;

		// Compute the loss w.r.t ground truth of a particular hypothesis
		double ComputeLoss(const ManhattanHypothesis& hypothesis) const;
	};

	class TrainingManager {
	public:
		ptr_vector<Map> maps;
		ptr_vector<proto::TruthedMap> gt_maps;
		ptr_vector<TrainingInstance> instances;

		map<string,Map*> maps_by_sequence;
		map<string,proto::TruthedMap*> gt_maps_by_sequence;

		// Feature generators
		LineSweepObjectiveGen objective_gen;  // for monocular payoffs
		ObjectivePayoffGen mono_gen;
		PointCloudPayoffs point_cloud_gen;
		StereoPayoffGen stereo_gen;  // one for each auxiliary frame

		// Load a sequence
		void LoadSequence(const string& sequence, vector<int> frame_ids);

		// Normalize features for mean and variance
		void NormalizeFeatures(FeatureManager& fmgr);

		// Get an instance
		// TODO: replace this by a generic to-python-sequence converter
		int NumInstances() const;
		TrainingInstance& GetInstance(int i);
	};

	// Helper class that performs MAP inference for manhattan models
	class ManhattanInference {
	public:
		ManhattanDPReconstructor reconstructor;
		const TrainingInstance* last_instance;

		// Constructor
		ManhattanInference() : last_instance(NULL) { }

		// Return values below will be invalidated by the next call to
		// either of the below
		ManhattanHypothesis Solve(const TrainingInstance& instance,
															const DPPayoffs& payoffs);

		// Get entities computed from the solution
		double GetSolutionScore() const;
		MatI GetSolutionLabels() const;
		MatD GetSolutionDepths();

		// Write a visualization of the solution to a file
		void OutputSolutionViz(const string& path) const;
	};

	// Manages loading / storing features to files
	class FeatureManager {
	public:
		string feature_dir;
		PayoffFeatures feature_set;
		const TrainingInstance* last_instance;

		LineSweepObjectiveGen objective_gen;
		ObjectivePayoffGen mono_gen;
		PointCloudPayoffs point_cloud_gen;
		StereoPayoffGen stereo_gen;

		FeaturePayoffGen payoff_gen;

		// Constructor for a particular dir
		FeatureManager(const string& dir);

		// Get the file for a specific instance
		string GetPathFor(const TrainingInstance& instance);
		// Get the i-th feature matrix
		MatF GetFeature(int i, int orient) const;
		// Get the description string associated with a feature
		const string& GetFeatureComment(int i) const;
		// Compile features into a payoff matrix
		VecD ComputeFeatureForHypothesis(const ManhattanHypothesis& soln) const;

		// Get the number of features
		int NumFeatures() const;
		// Write features to file
		void CommitFeatures();
		// Load features
		void LoadFeaturesFor(const TrainingInstance& instance);

		// Compute stereo, point cloud, and line sweep features
		void ComputeMultiViewFeatures(const TrainingInstance& instance,
																	const vector<int>& stereo_offsets);
		// Compute a host of photometric features. See PhotometricFeatures
		// for details about 'spec'
		void ComputeMonoFeatures(const TrainingInstance& instance,
														 const string& spec);
		// Compute a single feature for line sweeps (for ECCV comparison)
		void ComputeSweepFeatures(const TrainingInstance& instance);
		// Compute mock features for testing
		void ComputeMockFeatures(const TrainingInstance& instance);

		// Compile features
		void Compile(const ManhattanHyperParameters& params,
								 DPPayoffs& payoffs);
		void CompileWithLoss(const ManhattanHyperParameters& params,
												 const TrainingInstance& instance,
												 DPPayoffs& payoffs);
	};
}
