#pragma once

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
Vec3 HMidpoint(const Vec3& a,  const Vec3& b);

// Get the distance from a point to a line segment spanning two end
// points.
double GetLineSegDistance(const Vec2& line_a,
                          const Vec2& line_b,
                          const Vec2& point);

// Compute the euclidean distance between a 2D homogeneous point and line
double EuclPointLineDist(const Vec3& point, const Vec3& line);

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

// Get the depth of a plane at a particular pixel
double GetPlaneDepth(const Vec3& pixel, const toon::Matrix<3,4>& camera, const Vec4& plane);

// Intersect a ray with a plan
Vec3 IntersectRay(const Vec3& pixel, const toon::Matrix<3,4>& camera, const Vec4& plane);

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

}
