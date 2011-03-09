#include "common_types.h"

#include "manhattan_dp.h"

namespace indoor_context {
	class KeyFrame;
	class PosedCamera;

	class PointCloudPayoffs {
	public:
		const vector<Vec3>* input_points;
		const PosedCamera* input_camera;

		// This parameter is assigned from a gvar in the constructor. You
		// can change it after that.
		double agreement_sigma;

		DPGeometryWithScale geometry;

		MatF agreement_payoffs;
		MatF occlusion_payoffs;

		// Read gvar default values
		PointCloudPayoffs();
		// Compute payoff matrices for consistency with observed landmarks
		// Slow old implementation for comparison with Compute() only
		void Compute(const vector<Vec3>& points,
								 const PosedCamera& camera,
								 const DPGeometryWithScale& geom);
		// Compute payoff matrices for consistency with observed landmarks
		// Slow old implementation (only for comparison with new Compute())
		void ComputeSlow(const vector<Vec3>& points,
										 const PosedCamera& camera,
										 const DPGeometryWithScale& geom);
		// Compute payoff matrices for consistency with observed landmarks
		// Slow old implementation for comparison with Compute() only
		void ComputeSlow(const KeyFrame& frame,
								 const DPGeometryWithScale& geom);

		// Output points and the projections to floor/ceiling
		void OutputProjectionViz(const ImageBundle& image,
														 const string& path);
	};
}
