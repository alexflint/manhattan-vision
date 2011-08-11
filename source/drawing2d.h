#pragma once

#include "common_types.h"
#include "canvas.h"

namespace indoor_context {
	// Draw a set of axes
	void DrawAxes(Canvas& canvas, const PosedCamera& pc) {
		Vec2 image_ctr = pc.RetToIm(makeVector(0.0, 0.0));
		Vec3 scene_ctr = pc.pose_inverse() * makeVector(0,0,1);
		for (int j = 0; j < 3; j++) {
			Vec3 retina_ej = pc.pose() * (scene_ctr + GetAxis<3>(j));
			Vec2 image_ej = pc.RetToIm(project(retina_ej));
			DrawLineClipped(canvas, image_ctr, image_ej, BrightColors::Get(j));
		}
	}
}
