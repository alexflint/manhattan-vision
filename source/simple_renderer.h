/*
 * simple_renderer.h
 *
 * A minimalist 3D renderer with depth buffering and clipping.
 *
 *  Created on: 6 Jul 2010
 *      Author: alexf
 */

#pragma once

#include "common_types.h"

namespace indoor_context {

class SimpleRenderer {
public:
	SimpleRenderer();
	SimpleRenderer(const toon::Matrix<3,4>& cam, Vec2I viewport);
	// Configure the renderer with the given pose and viewport
	void Configure(const toon::Matrix<3,4>& cam, Vec2I viewport);
	// Render a triangle. Return true if at least one pixel was affected.
	bool Render(Vec3 p, Vec3 q, Vec3 r, int label);
	// Clear all buffers
	void Clear(int bg);
	// Get the frame buffer
	const MatI& framebuffer() const { return framebuffer_; }
	// Get the depth buffer
	const MatD& depthbuffer() const { return depthbuffer_; }
private:
	Vec2I viewport_;
	toon::Matrix<3,4> camera_;
	MatI framebuffer_;
	MatD depthbuffer_;
};


}  // namespace indoor_context
