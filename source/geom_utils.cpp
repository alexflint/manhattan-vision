#include <LU.h>
#include <SVD.h>

#include "geom_utils.h"
#include "common_types.h"
#include "camera.h"

#include "matrix_traits.tpp"
#include "vector_utils.tpp"
#include "vw_image.tpp"

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
		if (dpar < 0) {
			return max(d, norm(p-a));
		} else if (dpar > len) {
			return max(d, norm(p-b));
		}
	}

	double EuclPointLineDist(const Vec3& point, const Vec3& line) {
		return abs(SignedPointLineDist(point, line));
	}

	double SignedPointLineDist(const Vec3& point, const Vec3& line) {
		double nrm = sqrt(line[0]*line[0] + line[1]*line[1]);
		return (point * line) / (point[2] * nrm);
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
		SVD<3> svd(camera.slice<0,0,3,3>());
		Vec3 v = camera.T()[3];
		return svd.backsub(-v);
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

	Vec3 IntersectRay(const Vec3& pixel, const PosedCamera& camera, const Vec4& plane) {
		return IntersectRay(pixel, camera.Linearize(), plane);
	}

	Vec3 PlaneToDepthEqn(const Matrix<3,4>& camera, const Vec4& plane) {
		Mat4 M;
		M.slice<0, 0, 3, 4> () = camera;
		M.slice<3, 0, 1, 4> () = plane.as_row();
		SVD<4> decomp(M);
		double du = 1. / (camera[2] * atretina(decomp.backsub(makeVector(1,0,1,0))));
		double dv = 1. / (camera[2] * atretina(decomp.backsub(makeVector(0,1,1,0))));
		double dw = 1. / (camera[2] * atretina(decomp.backsub(makeVector(0,0,1,0))));
		return makeVector(du-dw, dv-dw, dw);
	}

	double EvaluateDepthEqn(const Vec3& depth_eqn, const Vec2& p) {
		return 1. / (depth_eqn*unproject(p));
	}

	double CrossRatio(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
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

	Mat3 ConstructPlanarHomology(const Vec3& vertex,
															 const Vec3& axis,
															 const Vec3& p,
															 const Vec3& q) {
		Mat3 I = Identity;
		double cr = CrossRatio(vertex, p, q, p^q^axis);
		double dp = vertex * axis;
		if (abs(dp) < 1e-8) {
			DLOG << "Warning: vertex * axis is close to zero ("<<dp<<") -- error in GetManhattanHomology?";
		}
		if (isnan(cr)) {
			DLOG << "Warning: cross ratio is NaN -- error in GetManhattanHomology?";
		} else if (abs(cr) > 1e+15) {
			DLOG << "Warning: cross ratio is very large ("<<cr<<") -- error in GetManhattanHomology?";
		} else if (abs(cr-1) < 1e-15) {
			DLOG << "Warning: cross ratio is very small ("<<cr<<") -- error in GetManhattanHomology?";
		}
		return I + (cr-1)*(vertex.as_col()*axis.as_row()) / dp;
	}

	Mat3 GetManhattanHomology(const Matrix<3,4>& cam, double z0, double z1) {
		Mat3 m = cam.slice<0,0,3,3>();
		Mat3 m_inv = LU<3>(m).get_inverse();

		Vec3 vertex = m * GetAxis<3>(2);
		Vec3 axis = m_inv.T() * GetAxis<3>(2);

		// TODO: X and Y position are arbitrary but it's important to pick
		// something away from the focal plane
		Vec3 p = cam * makeVector(10, -2, z0, 1);
		Vec3 q = cam * makeVector(10, -2, z1, 1);

		return ConstructPlanarHomology(vertex, axis, p, q);
	}

	Mat3 GetManhattanHomology(const PosedCamera& pc, double z0, double z1) {
		return GetManhattanHomology(pc.Linearize(), z0, z1);
	}

	Mat3 GetVerticalRectifierInRetina(const PosedCamera& pc) {
		Vec3 up = pc.GetRetinaVpt(2);
		if (up[1] < 0) up = -up;

		Mat3 R_up;
		R_up[0] = up ^ GetAxis<3>(2);
		R_up[1] = up;
		R_up[2] = R_up[0] ^ R_up[1];

		return R_up;
	}

	Mat3 GetVerticalRectifier(const PosedCamera& pc) {
		return GetVerticalRectifier(pc, pc.image_bounds());
	}

	Mat3 GetVerticalRectifier(const PosedCamera& pc,
														const Bounds2D<>& out_bounds) {
		Mat3 H_ret = GetVerticalRectifierInRetina(pc);

		// Compose it with the camera transform
		// NOTE: perhaps we could just use the image vanishing points and avoid this.
		Mat3 C = pc.camera().Linearize();
		Mat3 C_inv = LU<>(C).get_inverse();
		Mat3 H_rect = C * H_ret * C_inv;

		// Compute the image bounds after transformation
		Polygon<4> outline = pc.camera().image_bounds().GetPolygon();
		Polygon<4> rect_outline = outline.Transform(H_rect);
		Bounds2D<> rect_bounds = Bounds2D<>::ComputeBoundingBox(rect_outline);

		// Add a translation and scale to fit the image within the bounds
		double scale = (1-1e-8) * min(   // add a margin for roundoff tolerance
																	out_bounds.width() / rect_bounds.width(),
																	out_bounds.height() / rect_bounds.height());  // preserve aspect ratio
		Vec2 offset = out_bounds.center() - scale*rect_bounds.center();
		Mat3 H_fit = Identity;
		H_fit[0][0] = H_fit[1][1] = scale;
		H_fit.slice<0,2,2,1>() = offset.as_col();

		// Compose the transforms and do a sanity check
		Mat3 H = H_fit * H_rect;
		for (int i = 0; i < 4; i++) {
			CHECK_POS(project(H*outline.verts[i]), pc) << "i="<<i<<", outline[i]="<<outline.verts[i];
		}

		// Return the final transform
		return H;
	}

	Vec4 FitPlane(const vector<Vec3>& points) {
		Matrix<> m(points.size(), 4);
		for (int i = 0; i < points.size(); i++) {
			m.slice(i,0,1,4) = unproject(points[i]).as_row();
		}
		SVD<> svd(points.size(), 4);
		svd.compute(m);
		return svd.get_VT()[3];
	}


	// Rearrange and negate rows of a matrix so that it looks as much like
	// the identity as possible.
	Mat3 NormalizeRotation(const Mat3& R) {
		// This normalization makes sense for comparing rotation matrices
		// because if three rows of the matrix are mutually orthogonal then
		// any scalar multiplication of them must also be.
		Mat3 S = toon::Zeros;
		bool used[] = {false,false,false};
		for (int i = 0; i < 3; i++) {
			int ri = -1;
			for (int j = 0; j < 3; j++) {
				if (abs(R[j][i]) > S[i][i] && !used[j]) {
					S[i] = R[j] * Sign(R[j][i]);
					ri = j;
				}
			}
			used[ri] = true;
		}
		return S;
	}

	// Get a unit vector from spherical coordinates
	Vec3 SphericalToEuclidean(double theta, double psi) {
		return makeVector(cos(psi) * sin(theta),
											sin(psi) * sin(theta),
											cos(theta));
	}

	double RotationDistance(const Mat3& R0, const Mat3& R1) {
		Mat3 I = toon::Identity;
		return norm_fro(NormalizeRotation(R0 * R1.T()) - I);
	}
	double RotationDistance(const SO3<>& R0, const SO3<>& R1) {
		return RotationDistance(R0.get_matrix(), R1.get_matrix());
	}
}
