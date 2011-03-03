/*
 * floorplan_renderer.h
 * Renders a floorplan using simple software rendering
 *
 *  Created on: 1 Jun 2010
 *      Author: alexf
 */
#pragma once

#include "common_types.h"
#include "map.pb.h"
#include "simple_renderer.h"
#include "polygon.tpp"

namespace indoor_context {
class PosedCamera;
namespace proto { class FloorPlan; }

// Renders floorplans using a simple software renderer
class FloorPlanRenderer {
public:
	// Initialize empty
	FloorPlanRenderer();

	// Get the internal renderer
	SimpleRenderer& renderer() { return renderer_; }
	const SimpleRenderer& renderer() const { return renderer_; }

	// Render orientations
	void Render(const proto::FloorPlan& floorplan,
							const toon::Matrix<3,4>& camera,
							const Vec2I& viewport);
	void Render(const proto::FloorPlan& floorplan,
							const PosedCamera& camera);

	// Retrieve pixel-wise orientations
	MatI& orientations() { return renderer_.framebuffer(); }
	const MatI& orientations() const { return renderer_.framebuffer(); }

	// Retrieve pixel-wise depth
	MatD& depthmap() { return renderer_.depthbuffer(); }
	const MatD& depthmap() const { return renderer_.depthbuffer(); }

	// Retrieve quads for each wall in the model
	const vector<Polygon<4> >& walls() const { return walls_; }
	const vector<int>& wall_labels() const { return wall_labels_; }

	// Visualizations
	void DrawOrientations(ImageRGB<byte>& canvas);
private:
	SimpleRenderer renderer_;
	vector<Polygon<4> > walls_;
	vector<int> wall_labels_;
};

}  // namespace indoor_context
