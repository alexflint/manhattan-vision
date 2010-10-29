#pragma once

#include <iostream>

#include "common_types.h"
//#include "geom_utils.h"
#include "line_segment.h"
#include "polygon-fwd.h"

namespace indoor_context {

// Represents a base class
template <unsigned N, typename T>
class PolygonBase {
public:
	toon::Vector<3,T> verts[N];

	// Get an edge of the polygon
	LineSeg edge(int i) const {
		return LineSeg(verts[i], verts[(i+1)%N]);
	}

	// Matrix multiplication for polygon (take care with this, all sorts of weird
	// stuff can happen to polygons under projective transformations). Consider
	// the methods in clipping.h and clipping3d.h
	Polygon<N,T> Transform(const Mat3& m) {
		Polygon<N,T> p;
		for (int i = 0; i < N; i++) {
			p.verts[i] = m*verts[i];
		}
		return p;
	}
};

// Represents a polygon as a sequence of vertices
template <unsigned N, typename T/*=double*/>  // see polygon-fwd.h
class Polygon : public PolygonBase<N,T> {
};

// Specialization for line segments
template <typename T>
class Polygon<2,T> : public PolygonBase<2,T> {
public:
	Polygon() { }
	Polygon(const toon::Vector<3,T>& v0,
	        const toon::Vector<3,T>& v1) {
		PolygonBase<2,T>::verts[0] = v0;
		PolygonBase<2,T>::verts[1] = v1;
	}
};

// Specialization for triangles
template <typename T>
class Polygon<3,T> : public PolygonBase<3,T> {
public:
	Polygon() { }
	Polygon(const toon::Vector<3,T>& v0,
	        const toon::Vector<3,T>& v1,
	        const toon::Vector<3,T>& v2) {
		PolygonBase<3,T>::verts[0] = v0;
		PolygonBase<3,T>::verts[1] = v1;
		PolygonBase<3,T>::verts[2] = v2;
	}
};

// Specialization for quads
template <typename T>
class Polygon<4,T> : public PolygonBase<4,T>  {
public:
	Polygon() { }
	Polygon(const toon::Vector<3,T>& v0,
	        const toon::Vector<3,T>& v1,
	        const toon::Vector<3,T>& v2,
	        const toon::Vector<3,T>& v3) {
		PolygonBase<4,T>::verts[0] = v0;
		PolygonBase<4,T>::verts[1] = v1;
		PolygonBase<4,T>::verts[2] = v2;
		PolygonBase<4,T>::verts[3] = v3;
	}
};

// Represents an axis-aligned bounding box
template <typename T>  // see polygon-fwd.h
class Bounds2D {
	T left_, right_, top_, bottom_;
public:
	typedef toon::Vector<2,T> Vec2T;
	typedef toon::Vector<3,T> Vec3T;

	// Constructors
	Bounds2D() : left_(0), right_(0), top_(0), bottom_(0) { }
	Bounds2D(const T& left, const T& right, const T& top, const T& bottom)
	: left_(left), right_(right), top_(top), bottom_(bottom) { }

	// Accessors
	inline double left() const { return left_; }
	inline double right() const { return right_; }
	inline double top() const { return top_; }
	inline double bottom() const { return bottom_; }
	inline double width() const { return right_ - left_; }
	inline double height() const { return bottom_ - top_; }

	// Mutators
	inline void set_left(double v) { left_ = v; }
	inline void set_right(double v) { right_ = v; }
	inline void set_top(double v) { top_ = v; }
	inline void set_bottom(double v) { bottom_ = v; }

	// Get the coordinate of the corners
	Vec2T tl() const { return toon::makeVector(left_, top_); }
	Vec2T tr() const { return toon::makeVector(right_, top_); }
	Vec2T bl() const { return toon::makeVector(left_, bottom_); }
	Vec2T br() const { return toon::makeVector(right_, bottom_); }
	Vec2T center() const {
		return toon::makeVector((left_+right_)/2, (top_+bottom_)/2);
	}

	// Get the coordinate of the corners in homogeneous coordinates
	Vec3T htl() const { return toon::makeVector(left_, top_, 1); }
	Vec3T htr() const { return toon::makeVector(right_, top_, 1); }
	Vec3T hbl() const { return toon::makeVector(left_, bottom_, 1); }
	Vec3T hbr() const { return toon::makeVector(right_, bottom_, 1); }

	// Get homogeneous line equations for the boundaries. Interior of
	// this region is on the positive side of these lines.
	Vec3T left_eqn() const { return toon::makeVector(1,0,-left_); }
	Vec3T right_eqn() const { return toon::makeVector(-1,0,right_); }
	Vec3T top_eqn() const { return toon::makeVector(0,1,-top_); }
	Vec3T bottom_eqn() const { return toon::makeVector(0,-1,bottom_); }

	// Determine if the given point is inside these bounds
	template <typename U>
	bool Contains(const toon::Vector<2,U>& v) const {
		return v[0] >= left_ && v[0] <= right_ && v[1] >= top_ && v[1] <= bottom_;
	}

	// Determine if the given point is inside these bounds
	template <typename U>
	bool Contains(const toon::Vector<3,U>& v) const {
		return (v[0] >= v[2]*left_ && v[0] <= v[2]*right_ &&
				v[1] >= v[2]*top_  && v[1] <= v[2]*bottom_);
	}

	// Get a polygon representing the boundary of this region
	Polygon<4,T> GetPolygon() const {
		return Polygon<4,T>(htl(), htr(), hbr(), hbl());
	}

	// Construct bounds from any two opposing corners, in any order
	static Bounds2D<T> FromCorners(const Vec2& a, const Vec2& b) {
		return Bounds2D(min(a[0], b[0]), max(a[0], b[0]),
				min(a[1], b[1]), max(a[1], b[1]));
	}

	// Construct bounds from a size. Top-left corner is assumed to be at the origin.
	static Bounds2D<T> FromSize(const ImageRef& sz) {
		return Bounds2D(0, sz.x, 0, sz.y);
	}

	// Construct bounds from a size. Top-left corner is assumed to be at the origin.
	static Bounds2D<T> FromSize(const Vec2I& sz) {
		return Bounds2D(0, sz[0], 0, sz[1]);
	}

	// Construct bounds from a size. Top-left corner is assumed to be at the origin.
	static Bounds2D<T> FromTightSize(const ImageRef& sz) {
		return Bounds2D(0, sz.x-1, 0, sz.y-1);
	}
	// Construct bounds from a size. Top-left corner is assumed to be at the origin.
	static Bounds2D<T> FromTightSize(const Vec2I& sz) {
		return Bounds2D(0, sz[0]-1, 0, sz[1]-1);
	}

	// Compute the bounding box of a polygon
	template <unsigned N, typename S>
	static Bounds2D<T> ComputeBoundingBox(const Polygon<N,S>& poly) {
		Bounds2D<T> b(numeric_limits<T>::max(), numeric_limits<T>::min(),
				numeric_limits<T>::max(), numeric_limits<T>::min());
		for (int i = 0; i < N; i++) {
			Vec2 v = project(poly.verts[i]);
			if (v[0] < b.left_) b.left_ = v[0];
			if (v[0] > b.right_) b.right_ = v[0];
			if (v[1] < b.top_) b.top_ = v[1];
			if (v[1] > b.bottom_) b.bottom_ = v[1];
		}
		return b;
	}
};

// Stream output operator for Bounds2D
template <typename T>
ostream& operator<<(ostream& s, const Bounds2D<T>& b) {
	s << "{left= " << b.left() << ", right=" << b.right()
						<< ", top=" << b.top() << ", bottom=" << b.bottom() << "}";
}
}
