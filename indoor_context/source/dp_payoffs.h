#pragma once

#include "common_types.h"

namespace indoor_context {
	extern "C" lazyvar<float> gvDefaultWallPenalty;
	extern "C" lazyvar<float> gvDefaultOcclusionPenalty;

	////////////////////////////////////////////////////////////////////////////////
	// Represents a cost function that ManhattanDP optimizes, in terms of
	// the cost of assigning label A to pixel [Y,X] (stored in
	// pixel_scores[A][Y][X]). An object of this form is converted to the
	// more general representation of DPPayoffs
	class DPObjective;
	class DPObjective {
	public:
		double wall_penalty;  // the cost per wall segment (for regularisation)
		double occl_penalty;  // the cost per occluding wall segment 
		                      // (*in addition* to wall_penalty)
		MatF pixel_scores[3];  // score associated with assigning each
													 // label to each pixel (image coords)

		// Constructors
		DPObjective();
		DPObjective(const Vec2I& size);
		// Get size
		int nx() const { return pixel_scores[0].Cols(); }
		int ny() const { return pixel_scores[0].Rows(); }
		// Change size
		void Resize(const Vec2I& size);
		// Deep copy
		void CopyTo(DPObjective& rhs);
	private:
		// Disallow copy constructor (CopyTo explicitly)
		DPObjective(const DPObjective& rhs);
	};


	////////////////////////////////////////////////////////////////////////////////
	// The cost function the DP optimizes, specified in terms of the cost
	// of placing the top/bottom of a wall at each pixel.
	class DPPayoffs;
	class DPPayoffs {
	public:
		double wall_penalty;  // the cost per wall segment (for regularisation)
		double occl_penalty;  // the cost per occluding wall segment (_additional_ to wall_penalty)
		MatF wall_scores[2];  // cost of building the top/bottom of a wall at p (grid coords)

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
		void CopyTo(DPPayoffs& other);

		// Sum over a path (e.g. representing a candidate solution)
		double SumOverPath(const VecI& path, const VecI& orientations) const;

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
