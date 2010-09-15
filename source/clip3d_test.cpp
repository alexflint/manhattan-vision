#include "canvas.h"
#include "colors.h"

#include "viewer3d.h"
#include "widget3d.h"

#include "clipping3d.tpp"
#include "canvas.tpp"
#include "range_utils.tpp"
#include "io_utils.tpp"
#include "gl_utils.tpp"

using namespace std;
using namespace indoor_context;
using namespace toon;

void RenderPoly(const vector<Vec3>& poly, const PixelRGB<byte>& color) {
	glColorP(color);
	GL_PRIMITIVE(GL_POLYGON) {
		BOOST_FOREACH(const Vec3& v, poly) {
			glVertexV(v);
		}
	}
}

void DrawFrustrum(const PosedCamera& pc) {
	double znear = 1e-6, zfar = 1e+6;
	Polygon<4> poly = pc.retina_bounds().GetPolygon();
	GL_PRIMITIVE(GL_LINE_LOOP) {
		for (int i = 0; i < 4; i++) {
			glVertexV(poly.verts[i]*znear);
		}
	}
	GL_PRIMITIVE(GL_LINE_LOOP) {
		for (int i = 0; i < 4; i++) {
			glVertexV(poly.verts[i]*zfar);
		}
	}
	GL_PRIMITIVE(GL_LINES) {
		for (int i = 0; i < 4; i++) {
			glVertexV(poly.verts[i]*znear);
			glVertexV(poly.verts[i]*zfar);
		}
	}
}

int main(int argc, char **argv) {
	/*// Points on X axis
	Vec4 a = makeVector(1.0, 1, 5, 1);
	Vec4 b = makeVector(2.0, 2, 5, 1);

	// Plane
	Vec4 w = makeVector(1.0, 1, 0, -10);

	// XY and XZ planes
	Vec4 xy = makeVector(0.0, 0, 1, -5);
	Vec4 xz = makeVector(1.0, -1, 0, 0);

	Vec6 m = LineThrough(a,b);
	//Vec6 m = LineFromPlanes(xy,xz);
	Vec4 p = PlaneLineIsct(m,w);
	DREPORT(a,b,m,p,w);
	DREPORT(project(p));*/

	vector<Vec3> verts;
	verts.push_back(makeVector(0, -4, 1));
	verts.push_back(makeVector(4, 1, 1));
	verts.push_back(makeVector(4, 4, -1.0));

	// Camera intrinisics
	Mat3 mcam = Identity;
	Vec2 tr = makeVector(5.0, 5.0);
	mcam.slice<0,2,2,1>() = tr.as_col();

	// Camera extrinsics
	SE3<> pose;
	LinearCamera cam(mcam, ImageRef(10,10));
	PosedCamera pc(pose, cam);

	// Do the clipping
	vector<Vec3> fclipped;
	ClipToFrustrum(verts, pc, back_inserter(fclipped));
	DREPORT(fclipped.size());
	DREPORT(iowrap(fclipped));

	// Hack to help with drawing
	BOOST_FOREACH(Vec3& v, verts) {
		v[2] -= 0.1;
	}

	// Fire up the viewer
	Viewer3D v;
	v.AddOwned(new GroundPlaneWidget);
	v.Add(bind(&RenderPoly, ref(verts), Colors::blue()));
	v.Add(bind(&RenderPoly, ref(fclipped), Colors::red()));
	v.Add(bind(&DrawFrustrum, ref(pc)));
	v.Run();
	return 0;
}
