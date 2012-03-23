#include <TooN/so3.h>

#include "common_types.h"
#include "line_detector.h"

// Miscellaneous vizualization routines

namespace indoor_context {
	class PosedImage;

	// Draw line segments and vanishing points. Each line segment is
	// automatically associated with the vanishing point with minimal
	// reprojection error. The third parameter is a rotation of the
	// world.
	void OutputVptViz(const PosedImage& image,
										const vector<LineDetection>& detections,
										const toon::SO3<>& world_rotation,
										const string& path);

	// Identical to above
	void OutputVptViz(const PosedImage& image,
										const vector<LineSeg>& segments,
										const toon::SO3<>& world_rotation,
										const string& path);

	// Draw line segments and vanishing points, with specified
	// associations between lines and vanishing points. For element of
	// axes equal to -1, the corresponding line segment will be drawn
	// black with no vanishing point association.
	void OutputVptViz(const PosedImage& image,
										const vector<LineSeg>& segments,
										const vector<int>& axes,
										const toon::SO3<>& world_rotation,
										const string& path);
}
