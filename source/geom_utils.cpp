#include <LU.h>
#include <SVD.h>

#include "geom_utils.h"
#include "common_types.h"
#include "camera.h"

#include "math_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
using namespace toon;

// Local functions
namespace {
// Compute the determinant of a 2x2 matrix with the two vectors as its columns
inline double Det(const Vec2& a, const Vec2& b) {
	return a[0]*b[1] - b[0]*a[1];
}
}


bool PointInTriangle(const Vec2& a,
                     const Vec2& b,
                     const Vec2& c,
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
	double A = (a[0] - p[0]) * (b[1] - p[1]) - (b[0] - p[0]) * (a[1] - p[1]);
	double B = (b[0] - p[0]) * (c[1] - p[1]) - (c[0] - p[0]) * (b[1] - p[1]);
	double C = (c[0] - p[0]) * (d[1] - p[1]) - (d[0] - p[0]) * (c[1] - p[1]);
	double D = (d[0] - p[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (d[1] - p[1]);
	return Sign(A) == Sign(B) && Sign(B) == Sign(C) && Sign(C) == Sign(D);
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


double CrossRatio(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
	//return norm(a-c)*norm(b-d)/(norm(b-c)*norm(a-d));
	return Det(a,b)*Det(c,d) / (Det(a,c)*Det(b,d));
}

double CrossRatio(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
	return CrossRatio(project(a), project(b), project(c), project(d));
}

Mat3 GetHomographyVia(const Matrix<3,4>& from,
                      const Matrix<3,4>& to,
                      const Vec4& plane) {
	Mat4 m;
	m.slice<0,0,3,4>() = from;
	m.slice<3,0,1,4>() = plane.as_row();
	Mat4 m_inv = LU<4>(m).get_inverse();
	return to * m_inv.slice<0,0,4,3>();
}

Mat3 GetHomographyVia(const PosedCamera& from,
                      const PosedCamera& to,
                      const Vec4& plane) {
	return GetHomographyVia(from.Linearize(), to.Linearize(), plane);
}

Mat3 ConstructPlanarHomology(const Vec3& vertex, const Vec3& axis,
                             const Vec3& p, const Vec3& q) {
	Mat3 I = Identity;
	double cr = CrossRatio(vertex, p, q, p^q^axis);
	return I + (cr-1)*(vertex.as_col()*axis.as_row())/(vertex*axis);
}

Mat3 GetManhattanHomology(const Matrix<3,4>& cam, double z0, double z1) {
	Mat3 m = cam.slice<0,0,3,3>();
	Mat3 m_inv = LU<3>(m).get_inverse();

	Vec3 vertex = m * GetAxis<3>(2);
	Vec3 axis = m_inv.T() * GetAxis<3>(2);

	Vec3 p0 = cam * makeVector(0,0,z0,1);
	Vec3 p1 = cam * makeVector(0,0,z1,1);

	return ConstructPlanarHomology(vertex, axis, p0, p1);
}

Mat3 GetManhattanHomology(const PosedCamera& pc, double z0, double z1) {
	return GetManhattanHomology(pc.Linearize(), z0, z1);
}

Mat3 GetVerticalRectifier(const PosedCamera& pc) {
	return GetVerticalRectifier(pc, pc.image_bounds());
}

Mat3 GetVerticalRectifier(const PosedCamera& pc,
                          const Bounds2D<>& out_bounds) {
	// Here we assume that the Z axis represents the vertical direction
	Vec3 up = pc.GetRetinaVpt(2);
	if (up[1] < 0) up = -up;

	// Build a rotation to move the vertical vanishing point to infinity.
	// R must satisfy:
	//   (1) R*up = [0,1,0] or [0,-1,0]  (depending on sign of up[1]
	//   (2) R is as close to the identity as possible
	//        (so that the other vpts are minimally affected)
	Mat3 R_up;
	R_up[0] = up ^ GetAxis<3>(2);
	R_up[1] = up;
	R_up[2] = R_up[0] ^ R_up[1];

	// Compose it with the camera transform
	// NOTE: perhaps we could just use the image vanishing points and avoid this.
	Mat3 C = pc.camera().Linearize();
	Mat3 C_inv = LU<>(C).get_inverse();
	Mat3 H_rect = C * R_up * C_inv;

	// Compute the image bounds after transformation
	Polygon<4> outline = pc.camera().image_bounds().GetPolygon();
	Polygon<4> rect_outline = outline.Transform(H_rect);
	Bounds2D<> rect_bounds = Bounds2D<>::ComputeBoundingBox(rect_outline);

	// Add a translation and scale to fit the image within the bounds
	double scale = min(
			out_bounds.width() / rect_bounds.width(),
			out_bounds.height() / rect_bounds.height());  // preserve aspect ratio
	Vec2 offset = out_bounds.center() - scale*rect_bounds.center();
	Mat3 H_fit = Identity;
	H_fit[0][0] = H_fit[1][1] = scale;
	H_fit.slice<0,2,2,1>() = offset.as_col();

	// Return the final transform
	return H_fit * H_rect;
}
}
