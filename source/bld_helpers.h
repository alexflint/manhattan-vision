#include "common_types.h"

namespace indoor_context {
	class ManhattanReconstruction;
	class PosedCamera;

	namespace proto {
	class FloorPlan;
	class TruthedFrame;
	}

	// Get the ground truth orientation map for a frame by rendering the floorplan.
	void GetTrueOrients(const proto::FloorPlan& floorplan,
	                    const PosedCamera& pc,
	                    MatI& gt_orients);

	// Get the number of agreeing pixels
	int ComputeAgreement(const MatI& a, const MatI& b);
	// Get percentage of agreeing pixels
	double ComputeAgreementPct(const MatI& a, const MatI& n);

	// Downsample the orientation map to the specified size
	void DownsampleOrients(const MatI& in, MatI& out, const Vec2I& size);
	// Downsample the orientation map by a factor k
	void DownsampleOrients(const MatI& in, MatI& out, int k);

	// Replace all instances of A in m with B, and vice versa
	// Should no longer be needed
	void InterchangeLabels(MatI& m, int a, int b);

	// Get the floor->ceiling homology
	Mat3 GetFloorCeilHomology(const PosedCamera& pc, const proto::FloorPlan& fp);
}
