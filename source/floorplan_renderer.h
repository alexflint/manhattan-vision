/*
 * floorplan_renderer.h
 * Renders a floorplan using simple software rendering
 *
 *  Created on: 1 Jun 2010
 *      Author: alexf
 */
#include "common_types.h"

namespace indoor_context {
class PosedCamera;
namespace proto { class TruthedMap; }

class FloorplanRenderer {
public:
	FloorplanRenderer();
	FloorplanRenderer(const proto::TruthedMap& tru_map);
	void Configure(const proto::TruthedMap& tru_map);

	// Predict surface orientations at each pixel for a given camera
	void PredictOrients(const PosedCamera& pc, MatI& orients);
	// As above but rotate the labels
	void PredictSurfs(const PosedCamera& pc, MatI& orients);
private:
	vector<Vec4> planes;
	vector<int> plane_orients;
};

}  // namespace indoor_context
