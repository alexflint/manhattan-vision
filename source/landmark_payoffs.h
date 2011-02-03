#include "common_types.h"

namespace indoor_context {
	class KeyFrame;
	class PosedCamera;
	class DPGeometry;

	// Compute payoff matrices for consistency with observed landmarks
	void ComputeLandmarkPayoffs(const vector<Vec3> points,
															const PosedCamera& camera,
															const DPGeometry& geom,
															double zfloor,
															double zceil,
															MatF& agreement_payoffs,
															MatF& occl_payoffs);

	// Compute payoff matrices for consistency with observed landmarks
	void ComputeLandmarkPayoffs(const KeyFrame& frame,
															const DPGeometry& geom,
															double zfloor,
															double zceil,
															MatF& agreement_payoffs,
															MatF& occl_payoffs);
}
