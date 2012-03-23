#pragma once

#include "common_types.h"
#include "manhattan_dp.h"
#include "dp_payoffs.h"

#include "integral_image.tpp"

namespace indoor_context {
	class DPObjective;

	//////////////////////////////////////////////////////////////////
	// Computes payoffs from per-pixel features
	class FeaturePayoffGen {
	public:
		DPGeometry geom;
		IntegralColImage<float> integ_feature;
		DPPayoffs vpayoffs[2];  // feature sums for vertical intervals
		DPPayoffs hpayoffs;  // feature sums for floor/ceiling intervals
		MatF grid_buffer;

		// Initialize empty
		FeaturePayoffGen() { }
		// Compute payoffs
		void Compute(const MatF& feature, const DPGeometry& geom);
		// Return true iff not yet configured
		bool Empty() const;
		// Get payoff for building a wall at a point in grid coordinates.
		// grid_pt can be outside the grid bounds, clamping will be applied appropriately
		double GetVertSum(int x, int y) const;
		double GetHorizSum(int x, int y) const;
	};


	//////////////////////////////////////////////////////////////////
	// Computes payoffs from per-pixel labellings
	class ObjectivePayoffGen {
	public:
		DPGeometry geom;
		IntegralColImage<float> integ_scores[3];
		double wall_penalty, occl_penalty;
		DPPayoffs payoffs;  // payoffs are stored here when Compute() is called

		// Initialize empty
		ObjectivePayoffGen() { }
		// Initialize and compute
		ObjectivePayoffGen(const DPObjective& obj, const DPGeometry& geom);
		// Transform a DPObjective to a DPPayoff, storing the result in
		// this->payoffs. Equivalent to calling Configure(obj, geom) then
		// GetPayoffs(this->payoffs).
		void Compute(const DPObjective& obj,
								 const DPGeometry& geom);
		// Compute the integral images in preparation for calls to GetPayoff
		void Configure(const DPObjective& obj,
									 const DPGeometry& geom);
		// Return true iff not yet configured
		bool Empty() const;
		// Get payoff for building a wall at a point in grid coordinates.
		// grid_pt can be outside the grid bounds, clamping will be applied appropriately
		double GetWallScore(const Vec2& grid_pt, int orient) const;
		// Resize the matrix to the size of the grid and fill it with all payoffs
		void ComputePayoffs(DPPayoffs& payoffs) const;
	};
}
