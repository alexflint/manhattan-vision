#pragma once

#include <TooN/se3.h>
#include "common_types.h"

namespace indoor_context {
// Return true if the point P is inside the quad ABCD. Clockwise and
// anticlockwise are both fine.
bool PointInTriangle(const Vec2& a,
                     const Vec2& b,
                     const Vec2& c,
                     const Vec2& d,
                     const Vec2& p);

// Return true if the point P is inside the quad ABCD. Clockwise and
// anticlockwise are both fine.
bool PointInQuad(const Vec2& a,
                 const Vec2& b,
                 const Vec2& c,
                 const Vec2& d,
                 const Vec2& p);

// Clip a line to an image
void ClipLineToImage(Vec2& a,
                     Vec2& b,
                     const ImageRef& size);

// Find the (Euclidean) mid-point between two homogeneous vectors
Vec3 HMidpoint(const Vec3& a,
               const Vec3& b);

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
}
