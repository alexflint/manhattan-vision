#include "clipping3d.h"
#include "canvas.h"
#include "colors.h"

#include "canvas.tpp"
#include "range_utils.tpp"
#include "io_utils.tpp"
#include "gl_utils.tpp"

#include "viewer3d.h"
#include "widget3d.h"

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

	Polygon<4> square;
	square.verts[0] = makeVector(5, 0, 1);
	square.verts[1] = makeVector(10, 6, 1);
	square.verts[2] = makeVector(9, 10, 1);
	square.verts[3] = makeVector(0, 4, 1);

	Vec4 clip = makeVector(1.0, 1, 0, -10);

	vector<Vec3> clipped1;
	vector<Vec3> clipped2;
	ClipAgainstPlane(square.verts, clip, back_inserter(clipped1));
	ClipAgainstPlane(square.verts, -clip, back_inserter(clipped2));


	// Construct the camera
	Mat3 mcam = Identity;
	Vec2 tr = makeVector(5.0, 5.0);
	mcam.slice<0,2,2,1>() = tr.as_col();
	DREPORT(mcam * makeVector(0.0,0,1));
	DREPORT(mcam * makeVector(10.0,10,1));

	LinearCamera cam(mcam, ImageRef(10,10));

	SE3<> pose;
	//pose.get_rotation() = SO3<>(Identity);
	//pose.get_translation() = Zeros;
	DREPORT(pose);

	PosedCamera pc(pose, cam);

	vector<Vec3> fclipped;
	ClipAgainstFrustrum(square.verts, pc, back_inserter(fclipped));
	DREPORT(fclipped.size());
	DREPORT(iowrap(fclipped));

	Viewer3D v;
	v.AddOwned(new GroundPlaneWidget);
	v.Add(bind(&RenderPoly, fclipped, Colors::red()));
	v.Run();

	FileCanvas canvas("clipped.png", makeVector(250, 250));
	canvas.Scale(25);
	canvas.FillPolygon(clipped1, Colors::red());
	canvas.FillPolygon(clipped2, Colors::blue());

	return 0;
}
