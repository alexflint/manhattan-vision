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
	class PosedCamera;

	class SimpleRenderer {
	public:
		SimpleRenderer();
		// Initialize with the given camera
		SimpleRenderer(const PosedCamera& cam);
		// Initialize with the given camera and viewport
		SimpleRenderer(const toon::Matrix<3,4>& cam, Vec2I viewport);

		// Get the frame buffer
		const MatI& framebuffer() const { return framebuffer_; }
		MatI& framebuffer() { return framebuffer_; }
		// Get the depth buffer
		const MatD& depthbuffer() const { return depthbuffer_; }
		MatD& depthbuffer() { return depthbuffer_; }
		// Get the camera
		const toon::Matrix<3,4>& camera() const { return camera_; }
		// Get the viewport
		const Vec2I& viewport() const { return viewport_; }

		// Configure the renderer with the given camera and viewport
		void Configure(const PosedCamera& cam);
		// Configure the renderer with the given camera and viewport
		void Configure(const toon::Matrix<3,4>& cam, Vec2I viewport);
		// Clear all buffers
		void Clear(int bg);

		// Render a triangle. Return true if at least one pixel was affected.
		bool Render(const Vec2& p, const Vec2& q, const Vec2& r, int label);
		// Render a triangle (homogeneous coords). Return true if at least
		// one pixel was affected.
		bool Render(const Vec3& p, const Vec3& q, const Vec3& r, int label);
		// Render an infinite plane z=z0. Internally we just use very large extents.
		bool OldRenderInfinitePlane(double z0, int label);
		// Render an infinite plane z=z0. Internally we just use very large extents.
		bool RenderInfinitePlane(double z0, int label);

		// We often get NaN/inf pixels when there are walls close to the
		// horizon, or due to clipping issues near the boundary of the
		// image. As a simple work around we replace any such values by the
		// max finite value (which are clamped to kClampDepth). Returns the
		// number of pixels modified.
		int SmoothInfiniteDepths();
	private:
		Vec2I viewport_;
		toon::Matrix<3,4> camera_;
		MatI framebuffer_;
		MatD depthbuffer_;
	};
}  // namespace indoor_context
