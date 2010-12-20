#include "common_types.h"

namespace indoor_context {
	class KeyFrame;
	class DPGeometry;

	// Compute payoff matrices for consistency with observed landmarks
	void ComputeLandmarkPayoffs(const KeyFrame& frame,
															const DPGeometry& geom,
															double zfloor,
															double zceil,
															MatF& agreement_payoffs,
															MatF& occl_payoffs);
}
