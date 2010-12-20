/*
 * floorplan_renderer.h
 * Renders a floorplan using simple software rendering
 *
 *  Created on: 1 Jun 2010
 *      Author: alexf
 */
#include "common_types.h"
#include "map.pb.h"
#include "simple_renderer.h"

namespace indoor_context {
class PosedCamera;
namespace proto { class FloorPlan; }

class FloorPlanRenderer {
public:
	// Initialize empty
	FloorPlanRenderer();
	// Configure the renderer
	void Configure(const proto::FloorPlan& floorplan);

	// Get the internal renderer
	SimpleRenderer& renderer() { return renderer_; }
	const SimpleRenderer& renderer() const { return renderer_; }

	// Render orientations
	void Render(const proto::FloorPlan& floorplan,
							const toon::Matrix<3,4>& cam,
							const Vec2I& viewport);
	void Render(const proto::FloorPlan& floorplan,
							const PosedCamera& cam);

	// Retrieve the results
	const MatI& GetOrientations() { return renderer_.framebuffer(); }
	const MatD& GetDepthMap() const { return renderer_.depthbuffer(); }

	// Visualizations
	void DrawOrientations(ImageRGB<byte>& canvas);

private:
	SimpleRenderer renderer_;
};

}  // namespace indoor_context
