#include "common_types.h"
#include "map.pb.h"
#include "common_types.h"

namespace indoor_context {
	class ManhattanReconstruction;

	// Replace all instances of A in m with B, and vice versa
	void InterchangeLabels(MatI& m, int a, int b);

	// Load the ground truth orientation map for a frame. There are two
	// conventions for labelling the wall segments. If
	// label_by_tangents is true then they will be labelled by their
	// horizontal tangent direction. Otherwise they will be labelled by
	// their normal direction. These differ by swapping 0 and 1 labels.
	void LoadTrueOrients(const proto::TruthedFrame& tru_frame,
											 MatI& gt_orients,
											 bool label_by_tangents=true);

	// Get percentage of correct pixels
	double GetAccuracy(const MatI& estimated, const MatI& truth);

	// Get the number of agreeing pixels
	int GetAgreement(const MatI& a, const MatI& b);

	// Downsample the orientation map to the specified size
	void DownsampleOrients(const MatI& in, MatI& out, const toon::Vector<2,int>& size);
	// Downsample the orientation map by a factor k
	void DownsampleOrients(const MatI& in, MatI& out, int k);
}
