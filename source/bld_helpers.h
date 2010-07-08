#include "common_types.h"

namespace indoor_context {
	class ManhattanReconstruction;
	class PosedCamera;

	namespace proto {
	class FloorPlan;
	class TruthedFrame;
	}

	// Replace all instances of A in m with B, and vice versa
	void InterchangeLabels(MatI& m, int a, int b);

	// Get the ground truth orientation map for a frame by rendering the floorplan.
	// There are two conventions for labelling the wall segments. If
	// label_by_tangents is true then they will be labelled by their
	// horizontal tangent direction. Otherwise they will be labelled by
	// their normal direction. These differ by swapping 0 and 1 labels.
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

	// Load the ground truth orientation map for a frame from a file. There are two
	// conventions for labelling the wall segments. If
	// label_by_tangents is true then they will be labelled by their
	// horizontal tangent direction. Otherwise they will be labelled by
	// their normal direction. These differ by swapping 0 and 1 labels.
	void LoadTrueOrientsOld(const proto::TruthedFrame& tru_frame,
	                        MatI& gt_orients,
	                        bool label_by_tangents=true);
}
