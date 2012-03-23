#pragma once

#include "common_types.h"

namespace indoor_context {
	extern "C" lazyvar<float> gvDefaultWallPenalty;
	extern "C" lazyvar<float> gvDefaultOcclusionPenalty;

	/////////////////////////////////////////////////////////////////////
	// Represents a cost function that ManhattanDP optimizes, specified
	// in terms of the cost of placing the top/bottom of a wall at any
	// given pixel.
	class DPPayoffs;
	class DPPayoffs {
	public:
		double wall_penalty;  // the cost per wall segment (for regularisation)
		double occl_penalty;  // the cost per occluding wall segment (added to wall_penalty)
		MatF wall_scores[2];  // cost of placing the top/bottom of a wall at p (grid coords)

		// Initialize empty
		DPPayoffs();
		// Initialize and allocate (and initiale all payoffs to zero)
		DPPayoffs(Vec2I size);
		// Get width
		int nx() const { return wall_scores[0].Cols(); }
		// Get Height
		int ny() const { return wall_scores[0].Rows(); }

		// Resize the score matrix and initialze to zero
		void Resize(Vec2I size);
		// Resize the score matrix and reset all elements to the specified value
		void Clear(float fill);
		// Clone this object
		void CopyTo(DPPayoffs& other) const;

		// Compute score for a hypothesis
		double ComputeScore(const VecI& path,
												const VecI& axes,
												int num_walls,
												int num_occlusions) const;
		// Sum these payoffs over a path. Equal to above without penalty terms.
		double SumOverPath(const VecI& path, const VecI& axes) const;

		// Add a payoff matrix weighted by a constant
		void Add(const DPPayoffs& other, double weight=1.0);
		// Add a payoff matrix to both wall_scores[0] and wall_scores[1],
		// multiplied by a constant.
		void Add(const MatF& delta, double weight=1.0);
	private:
		// Disallow copy constructor (use CopyTo explicitly instead)
		DPPayoffs(const DPPayoffs& rhs);
	};
}
