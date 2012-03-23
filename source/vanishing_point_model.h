// These classes supercede those in vanishing_points.* and
// rotation_estimator.*. Here we use the geometric reprojection error
// and apply the Generalized EM algorithm. We also have a cleaner
// separation of model and optimizer.

#pragma once

#include <TooN/so3.h>

#include "common_types.h"
#include "line_segment.h"

namespace indoor_context {
	class LineSegment;
	class PosedCamera;
	class VanishingPointModel;

	///////////////////////////////////////////////////////////////////////////
	// Simple container for line observations
	class LineObservation {
	public:
		const PosedCamera* camera;
		LineSegment segment;

		LineObservation() : camera(NULL) { }
		LineObservation(const PosedCamera* cam, const LineSegment& seg)
			: camera(cam), segment(seg) { }
	};

	///////////////////////////////////////////////////////////////////////////
	typedef vector<pair<LineObservation,int> > CompleteData;

	///////////////////////////////////////////////////////////////////////////
	class VanishingPointModel {
	public:
		double line_sigma_sqr;
		VanishingPointModel(double line_ss) : line_sigma_sqr(line_ss) {
		}

		//
		// Likelihoods
		//

		// Compute the log likelihood of a line segment given the location of
		// its associated vanishing point.
		double ComputeSegmentLogLik(const LineSegment& seg, const Vec3& vpt) const;
		// Compute the log likelihood of a line segment given its label
		double ComputeObservationLogLik(const LineObservation& obs,
																		int label,
																		const toon::SO3<>& R) const;
		// Compute log likelihood of a set of line segments and labels
		double ComputeCompleteLogLik(const CompleteData& data,
																 const toon::SO3<>& R) const;
		// Compute expected log likelihood under the specified posterior
		// on latent variables
		double ComputeExpectedLogLik(const vector<LineObservation>& data,
																 const MatD& responsibilities,
																 const toon::SO3<>& R) const;

		//
		// Posteriors
		//
		void ComputePosteriorOnLabels(const vector<LineObservation>& data,
																	const toon::SO3<>& R,
																	MatD& out_responsibilities) const;

		//
		// Gradients
		//
		Vec3 ComputeDistGradient(const LineSegment& seg,
														 int axis,
														 const PosedCamera& camera,
														 const toon::SO3<>& Rcur) const;
		Vec3 ComputeSegmentLogLikGradient(const LineSegment& seg,
																			int axis,
																			const PosedCamera& camera,
																			const toon::SO3<>& Rcur) const;
		Vec3 ComputeObservationLogLikGradient(const LineObservation& obs,
																					int axis,
																					const toon::SO3<>& Rcur) const;
		Vec3 ComputeCompleteLogLikGradient(const CompleteData& data,
																			 const toon::SO3<>& Rcur) const;
		Vec3 ComputeExpectedLogLikGradient(const vector<LineObservation>& data,
																			 const MatD& responsibilities,
																			 const toon::SO3<>& Rcur) const;

		// Numeric gradients for verification
		Vec3 ComputeCompleteLogLik_NumericGradient(const CompleteData& data,
																							 const toon::SO3<>& Rcur,
																							 double delta=1e-6) const;
		Vec3 ComputeExpectedLogLik_NumericGradient(const vector<LineObservation>& data,
																							 const MatD& responsibilities,
																							 const toon::SO3<>& Rcur,
																							 double delta) const;
	};

	///////////////////////////////////////////////////////////////////////////
	class RotationOptimizer {
	public:
		// Parameters (can be changed after construction)
		int max_iters;
		double min_improvement;
		double min_step_norm;

		// The function we're optimizing
		boost::function<double(const toon::SO3<>&)> f;
		boost::function<Vec3(const toon::SO3<>&)> Jf;

		// Updated on each call to Step()
		toon::SO3<> Rcur;    // current point
		double fcur;         // likelihood at this point
		Vec3 Gcur;           // gradient at current point
		int num_steps;       // total steps taken so far
		double improvement;  // size of last improvement
		Vec3 last_step;      // the last gradient step (incl. effect of step_size)
		double step_norm;    // norm of last step (incl. effect of step_size)

		bool initialized;
		double step_size;    // this is multiplied by Gcur to get step_norm

		// Data an the evolution of the optimization. Deleted by
		// Initialize(), useful for analysis.
		vector<toon::SO3<> > Rs;    // rotation at each step
		vector<double> fs;          // objective func at each step
		vector<Vec3> gradients;     // gradient at each step
		vector<double> step_sizes;

		// Constructor
		RotationOptimizer();

		toon::SO3<> OptimizeLogLik(const VanishingPointModel& model,
															 const CompleteData& data,
															 const toon::SO3<>& R_init);

		// Take steps until convergence or timeout
		toon::SO3<> Compute(const toon::SO3<>& R_init);		
		// Initialize at some point
		void Initialize(const toon::SO3<>& R);
		// Take a gradient step, return the step taken (including scaling by step_size)
		Vec3 Step();
		// True if Initialize(...) has been called at least once
		bool Initialized();
		// Check for convergence
		bool Converged();
		// Helper to compute f, its gradient, and store them
		void MoveTo(const toon::SO3<>& R);
		void MoveTo(const toon::SO3<>& R, double fR);
		// Helper to add values to history
		void RecordState();
	};

	////////////////////////////////////////////////////////////////////////////////
	// Implements EM for vanishing point estimation. Uses
	// RotationOptimizer for the M step and VanishingPointModel for the
	// E step.
	class VanishingPointEstimator {
	public:
		const vector<LineObservation>* observations;
		const VanishingPointModel* model;
		RotationOptimizer optimizer;

		scoped_ptr<VanishingPointModel> owned_model;  // for managing memory only
		toon::SO3<> R_cur;   // current estimate

		// Updated on each iteration
		MatD responsibilities;
	
		// Constructors
		VanishingPointEstimator();
		VanishingPointEstimator(const vector<LineObservation>& observations);
		VanishingPointEstimator(const vector<LineObservation>& observations,
														const VanishingPointModel& model);

		// Sigma_sqr is the variance of the reprojection error model
		void Configure(const vector<LineObservation>& observations);
		void Configure(const vector<LineObservation>& observations,
									 const VanishingPointModel& model);

		// Iterate to convergence
		toon::SO3<> Compute(const toon::SO3<>& Rinit);
		// Update responsibilities
		void EStep();
		// Take a gradient step
		void MStep();
		// Test for convergence
		bool Converged();
		// Get the current estimate
		toon::SO3<> CurrentEstimate();
	};
}
