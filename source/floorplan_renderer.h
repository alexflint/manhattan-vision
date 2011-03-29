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
#include "model.pb.h"
#include "simple_renderer.h"
#include "vw_image-fwd.h"

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

	// Configure the camera
	void Configure(const PosedCamera& camera);
	void Configure(const toon::Matrix<3,4>& camera, const Vec2I& viewport);

	// Reset with just the infinite floor and ceiling planes
	void Reset(double zfloor, double zceil);

	// Get the binary orientation label for a wall
	int GetWallOrientation(const Vec2& u, const Vec2& v);

	// Render orientations
	void Render(const proto::FloorPlan& floorplan);
	void Render(const proto::Model& model);
	void RenderWall(const Vec2& u, const Vec2& v, double zfloor, double zceil);

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
