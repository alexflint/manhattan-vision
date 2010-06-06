#include <cv.h>
#include <SVD.h>

#include "geom_utils.h"
#include "common_types.h"

#include "math_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
using namespace toon;

bool PointInTriangle(const Vec2& a,
                     const Vec2& b,
                     const Vec2& c,
                     const Vec2& d,
                     const Vec2& p) {
	double A, B, C;
	A = (a[0] - p[0]) * (b[1] - p[1]) - (b[0] - p[0]) * (a[1] - p[1]);
	B = (b[0] - p[0]) * (c[1] - p[1]) - (c[0] - p[0]) * (b[1] - p[1]);
	C = (c[0] - p[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (c[1] - p[1]);
	return Sign(A) == Sign(B) && Sign(B) == Sign(C);
}

bool PointInQuad(const Vec2& a,
                 const Vec2& b,
                 const Vec2& c,
                 const Vec2& d,
                 const Vec2& p) {
	double A, B, C, D;
	A = (a[0] - p[0]) * (b[1] - p[1]) - (b[0] - p[0]) * (a[1] - p[1]);
	B = (b[0] - p[0]) * (c[1] - p[1]) - (c[0] - p[0]) * (b[1] - p[1]);
	C = (c[0] - p[0]) * (d[1] - p[1]) - (d[0] - p[0]) * (c[1] - p[1]);
	D = (d[0] - p[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (d[1] - p[1]);
	return Sign(A) == Sign(B) && Sign(B) == Sign(C) && Sign(C) == Sign(D);
}

void ClipLineToImage(Vec2& a,
                     Vec2& b,
                     const ImageRef& size) {
	CvSize sz = cvSize(size.x, size.y);
	CvPoint cva = cvPoint(a[0], a[1]);
	CvPoint cvb = cvPoint(b[0], b[1]);
	cvClipLine(sz, &cva, &cvb);
	a[0] = cva.x;
	a[1] = cva.y;
	b[0] = cvb.x;
	b[1] = cvb.y;
}

Vec3 HMidpoint(const Vec3& a, const Vec3& b) {
	return makeVector(a[0]*b[2] + b[0]*a[2],
			a[1]*b[2] + b[1]*a[2],
			2*a[2]*b[2]);
}

double GetLineSegDistance(const Vec2& a,
                          const Vec2& b,
                          const Vec2& p) {
	double len = norm(b-a);
	Vec2 tang = makeVector(b[0]-a[0], b[1]-a[1]) / len;
	Vec2 nrm = makeVector(-tang[1], tang[0]);
	double d = abs((p-a)*nrm);
	double dpar = (p-a)*tang;
	if (dpar < 0) d = max(d, norm(p-a));
	else if (dpar > len) d = max(d, norm(p-b));
	return d;
}

double EuclPointLineDist(const Vec3& point, const Vec3& line) {
	// Take the dot product and normalize by the last element of the
	// point and the first two elements of the line
	double nrm = line[0]*line[0] + line[1]*line[1];
	return abs((point * line) / (point[2] * nrm));
}

double HNormSq(const Vec3& a, const Vec3& b) {
	double dx = a[0]*b[2] - b[0]*a[2];
	double dy = a[1]*b[2] - b[1]*a[2];
	double zz = a[2]*b[2];
	return (dx*dx + dy*dy) / (zz*zz);
}

double HNorm(const Vec3& a, const Vec3& b) {
	return sqrt(HNormSq(a, b));
}

Mat3 BuildRotation(const Vec3& axis,
                   const double& sin_t,
                   const double& cos_t) {
	// doesn't seem to work
	Vec3 u = unit(axis);
	Mat3 skew;
	skew[0] = makeVector(0, -u[2], u[1]);
	skew[1] = makeVector(u[2], 0, -u[0]);
	skew[2] = makeVector(-u[1], u[0], 0);
	Mat3 outer = u.as_col()*u.as_row();
	return outer + cos_t*(Identity-outer) + sin_t*skew;
}

Mat3 BuildRotation(const Vec3& u, const double& t) {
	return BuildRotation(u, sin(t), cos(t));
}

Vec3 GetCameraCentre(const Matrix<3,4>& camera) {
	// The camera centre X is given by -inv(C)*t where C is the
	// leftmost 3x3 cells of the camera matrix and t is the rightmost
	// column.
	return SVD<3>(camera.slice<0, 0, 3, 3> ()).backsub(-camera.T()[3]);
}

double GetPlaneDepth(const Vec3& p, const Matrix<3,4>& camera, const Vec4& plane) {
	Mat4 M;
	M.slice<0, 0, 3, 4> () = camera;
	M.slice<3, 0, 1, 4> () = plane.as_row();
	SVD<4> decomp(M);
	return camera[2] * atretina(decomp.backsub(concat(p, 0.0)));
}

Vec3 IntersectRay(const Vec3& p, const Matrix<3,4>& camera, const Vec4& plane) {
	Mat4 M;
	M.slice<0, 0, 3, 4> () = camera;
	M.slice<3, 0, 1, 4> () = plane.as_row();
	SVD<4> decomp(M);
	return project(decomp.backsub(makeVector(p[0], p[1], p[2], 0.0)));
}

Vec3 PlaneToDepthEqn(const Matrix<3,4>& camera, const Vec4& plane) {
	Mat4 M;
	M.slice<0, 0, 3, 4> () = camera;
	M.slice<3, 0, 1, 4> () = plane.as_row();
	SVD<4> decomp(M);
	double du = 1.0 / (camera[2] * atretina(decomp.backsub(makeVector(1,0,1,0))));
	double dv = 1.0 / (camera[2] * atretina(decomp.backsub(makeVector(0,1,1,0))));
	double dw = 1.0 / (camera[2] * atretina(decomp.backsub(makeVector(0,0,1,0))));
	return makeVector(du-dw, dv-dw, dw);
}
}
