#pragma once

#include "common_types.h"

namespace indoor_context {
	// TODO rename DPObjective -> DPAffinities, remove *_penalty

	////////////////////////////////////////////////////////////////////
	// Represents a cost function that ManhattanDP optimizes, in terms
	// of the cost of assigning label A to pixel [Y,X] (stored in
	// pixel_scores[A][Y][X]). An object of this form is converted to
	// the more general representation of DPPayoffs
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
}
