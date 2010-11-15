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

	// Render from a particular view
	void Render(const proto::FloorPlan& floorplan,
	            const toon::Matrix<3,4>& cam,
	            const Vec2I& viewport,
	            ImageRGB<byte>& canvas);
	void Render(const proto::FloorPlan& floorplan,
	            const PosedCamera& cam,
	            ImageRGB<byte>& canvas);

	// Render orientations
	void RenderOrients(const proto::FloorPlan& floorplan,
	                   const toon::Matrix<3,4>& cam,
	                   const Vec2I& viewport,
	                   MatI& orients);
	void RenderOrients(const proto::FloorPlan& floorplan,
	                   const PosedCamera& cam,
										 MatI& orients);
	void RenderOrients(const proto::FloorPlan& floorplan,
	                   const PosedCamera& cam,
										 MatI& orients,
										 MatD& depthmap);
private:
	SimpleRenderer renderer_;

	// Send the floorplan to the renderer
	void RenderInternal(const proto::FloorPlan& floorplan,
	                    const toon::Matrix<3,4>& cam,
	                    const Vec2I& viewport);
};

}  // namespace indoor_context
