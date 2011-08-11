#include <GL/gl.h>

#include <boost/foreach.hpp>

#include "hotspot.h"
#include "viewer3d.h"
#include "widget3d.h"

#include "gl_utils.tpp"

namespace indoor_context {
using namespace toon;

void HotSpots::Draw() {
	WITHOUT(GL_DEPTH_TEST) {
		BOOST_FOREACH(HotSpot& spot, spots) {
			spot.screenPos = viewer().ProjectToScreen(spot.worldPos);
			if (hovering()) {
				glPointSize(kDefaultRadius);
				glColor3f(0,0,0);
				GL_PRIMITIVE(GL_POINTS) glVertexV(spot.worldPos);
				glPointSize(kDefaultRadius-2);
				glColor3f(0,1,0);
				GL_PRIMITIVE(GL_POINTS) glVertexV(spot.worldPos);
			}
		}
	}
}

ostream& HotSpots::Add(Vector<3> p, float rad) {
	HotSpot spot;
	spot.worldPos = p;
	spot.radius = rad;
	spot.text.reset(new ostringstream);
	spots.push_back(spot);
	return *spot.text;
}

bool HotSpots::HitTest(const Vector<2>& mouse) const {
	BOOST_FOREACH(const HotSpot& h, spots) {
		if (norm_sq(h.screenPos-mouse) < h.radius*h.radius) {
			return true;
		}
	}
	return false;
}

void HotSpots::OnClick(int button, int x, int y) {
	Vector<2> mouse = makeVector(x,y);
	BOOST_FOREACH(const HotSpot& h, spots) {
		if (norm_sq(h.screenPos-mouse) < h.radius*h.radius) {
			cout << "\n=======\n" << h.text->str() << "\n=======\n";
		}
	}
}
}
