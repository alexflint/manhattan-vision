/*
 * colored_points.h
 *
 *  Created on: 14 May 2010
 *      Author: alexf
 */

#pragma once

#include <vector>
#include <GL/gl.h>

#include "common_types.h"
#include "widget3d.h"

#include "gl_utils.tpp"

namespace indoor_context {

class ColoredPoints : public Widget3D {
public:
	double pointSize;
	vector<pair<Vec3, PixelRGB<byte> > > vs;
	ColoredPoints();

	// Add a point
	void Add(const Vec3& v, const PixelRGB<byte>& color);

	// Render the points
	virtual void OnRender() {
		glPointSize(pointSize);
		GL_PRIMITIVE(GL_POINTS) {
			for (int i = 0; i < vs.size(); i++) {
				glColorP(vs[i].second);
				glVertexV(vs[i].first);
			}
		}
	}
};

}  // namespace indoor_context
