#pragma once

#include <TooN/so3.h>

#include "common_types.h"
#include "polygon-fwd.h"

namespace indoor_context {
	// Forward declarations
	class PosedCamera;

	// Return true if the point P is inside the quad ABCD. Clockwise and
	// anticlockwise are both fine.
	bool PointInTriangle(const Vec2& a,
											 const Vec2& b,
											 const Vec2& c,
											 const Vec2& p);

	// Return true if the point P is inside the quad ABCD. Clockwise and
	// anticlockwise are both fine.
	bool PointInQuad(const Vec2& a,
									 const Vec2& b,
									 const Vec2& c,
									 const Vec2& d,
									 const Vec2& p);

	// Find the (Euclidean) mid-point between two homogeneous vectors
	Vec3 HMidpoint(const Vec3& a, const Vec3& b);

	// Get the distance from a point to a line segment spanning two end
	// points.
	double GetLineSegDistance(const Vec2& seg_start,
														const Vec2& seg_seg_end,
														const Vec2& point);

	// Compute the euclidean distance between a 2D homogeneous point and line
	double EuclPointLineDist(const Vec3& point, const Vec3& line);

	// Signed version of EuclPointLineDist. The sign is positive on one
	// side of the line and negative on the other side.
	double SignedPointLineDist(const Vec3& point, const Vec3& line);

	// Determine Euclidean distance between two homogeneous vectors
	double HNormSq(const Vec3& a, const Vec3& b);

	// Determine Euclidean distance between two homogeneous vectors
	double HNorm(const Vec3& a, const Vec3& b);

	// Get the matrix for a rotation about an axis u by an angle t
	// This doesn't seem to work, oops
	Mat3 BuildRotation(const Vec3& axis,
										 const double& sin_t,
										 const double& cos_t);

	// Get the matrix for a rotation about an axis u by an angle t
	Mat3 BuildRotation(const Vec3& u, const double& t);

	// Get a camera centre in world coordinates
	Vec3 GetCameraCentre(const toon::Matrix<3,4>& camera);

	// Find an equation relating the (x,y) coordinates in an image to the
	// reciprocal of the depth of a given plane at that pixel. Important: depth
	// at pixel (x,y) is 1.0/(eqn*makeVector(x,y,1)), where eqn is the return value
	// from this function
	Vec3 PlaneToDepthEqn(const toon::Matrix<3,4>& camera, const Vec4& plane);

	// Evaluate a depth equation as returned from PlaneToDepthEquation
	// at a particular image location.
	double EvaluateDepthEqn(const Vec3& depth_eqn, const Vec2& p);

	// Get the depth of a plane at a particular pixel
	double GetPlaneDepth(const Vec3& pixel, const toon::Matrix<3,4>& camera, const Vec4& plane);

	// Intersect a ray with a plane
	Vec3 IntersectRay(const Vec3& pixel, const toon::Matrix<3,4>& camera, const Vec4& plane);
	Vec3 IntersectRay(const Vec3& pixel, const PosedCamera& camera, const Vec4& plane);

	// Compute the cross ratio for four points in homgeneous coordinates
	// See Hartley&Zisserman
	double CrossRatio(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d);

	// Compute the cross ratio for four points in homgeneous coordinates
	// See Hartley&Zisserman
	double CrossRatio(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);

	// Get the homography mapping points from one camera, onto a plane,
	// then into another camera.
	Mat3 GetHomographyVia(const toon::Matrix<3,4>& from,
												const toon::Matrix<3,4>& to,
												const Vec4& plane);

	// Get the homography mapping points from one camera, onto a plane,
	// then into another camera.
	Mat3 GetHomographyVia(const PosedCamera& from,
												const PosedCamera& to,
												const Vec4& plane);

	// Construct the planar homology M with the given vertex, axis, and pair of points:
	//   M*p = q
	//   M*vertex = vertex
	//   M*x = x  for all x on axis (i.e. fo all x such that x*axis=0)
	// See Criminisi "Single View Metrology"
	Mat3 ConstructPlanarHomology(const Vec3& vertex,
															 const Vec3& axis,
															 const Vec3& p,
															 const Vec3& q);

	// Get a mapping from points on the plane {z=z0} to the plane {z=z1},
	// projected in the specified camera.
	Mat3 GetManhattanHomology(const toon::Matrix<3,4>& cam, double z0, double z1);

	// Get a mapping from points on the plane z=z0 to the plane z=z1,
	// projected in the specified camera.
	Mat3 GetManhattanHomology(const PosedCamera& pc, double z0, double z1);

	// Get a homogrphy that makes vertical lines in the world appear vertical in the image.
	// The output image will fit within the bounds of the original image.
	// (aspect ratio will be preserved as much as possible).
	Mat3 GetVerticalRectifier(const PosedCamera& pc);

	// As above, but the output image will fit within the specified bounds
	// (aspect ratio will be preserved as much as possible).
	Mat3 GetVerticalRectifier(const PosedCamera& pc, const Bounds2D<>& out_bounds);

	// Get a homogrphy that makes vertical lines in the world appear
	// vertical in the camera frame (working in retina coordinates).
	Mat3 GetVerticalRectifierInRetina(const PosedCamera& pc);

	// Fit a least--squares plane to a set of points.
	// The normal vector can be recovered as
	//    Vec3 normal = unit(plane.slice<0,3>());
	// where plane is the return value from this function
	Vec4 FitPlane(const vector<Vec3>& points);

	// Get a unit vector from spherical coordinates
	Vec3 SphericalToEuclidean(double theta, double psi);

	// Rearrange and possibly negate rows of a matrix so that it looks
	// as much like the identity as possible.
	Mat3 NormalizeRotation(const Mat3& R);

	// Compute a metric in the space of rotation matrices. It's hard to
	// see exactly what the "right" comparison of rotations is, but
	// luckily there is a paper showing that most commonly used metrics
	// are just monotonic transformations of one another, and this (very
	// simple) metric is one, meaning it is "probably a reasonable
	// metric" for examining intermediate errors, etc.
	//
	// See: Huynh, "Metrics for 3D Rotations: Comparison and Analysis", 2009
	double RotationDistance(const Mat3& R0, const Mat3& R1);
	double RotationDistance(const toon::SO3<>& R0, const toon::SO3<>& R1);
}
