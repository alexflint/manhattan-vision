#pragma once

#include "common_types.h"
#include "geom_utils.h"
#include "polygon-fwd.h"

namespace indoor_context {
	// Represents a 2D line segment in homogeneous coordinates
	class LineSeg {
	public:
		toon::Vector<3> start, end;

		inline LineSeg()
			: start(toon::Zeros), end(toon::Zeros) {
		}
		inline LineSeg(const toon::Vector<3>& a,
									 const toon::Vector<3>& b)
			: start(a), end(b) { 
		}
		// Get the homogeneous line equation
		inline toon::Vector<3> eqn() const {
			return start^end;
		}
		// Get the midpoint (in the Euclidean sense)
		inline toon::Vector<3> midpoint() const {
			return HMidpoint(start, end);
		}
	};

	// Represents a polygon as a sequence of vertices
	template <unsigned N, typename T/*=double*/>  // see polygon-fwd.h
	class Polygon	{
	public:
		toon::Vector<3,T> verts[N];

		LineSeg edge(int i) {
			return LineSeg(verts[i], verts[(i+1)%N]);
		}
	};

	// Specialization for line segments
	template <typename T>
	class Polygon<2,T> {
	public:
		Polygon() { }
		Polygon(const toon::Vector<3,T>& v0,
						const toon::Vector<3,T>& v1) {
			verts[0] = v0;
			verts[1] = v1;
		}
		toon::Vector<3,T> verts[2];
	};

	// Specialization for triangles
	template <typename T>
	class Polygon<3,T> {
	public:
		Polygon() { }
		Polygon(const toon::Vector<3,T>& v0,
						const toon::Vector<3,T>& v1,
						const toon::Vector<3,T>& v2) {
			verts[0] = v0;
			verts[1] = v1;
			verts[2] = v2;
		}
		toon::Vector<3,T> verts[3];
	};

	// Specialization for quads
	template <typename T>
	class Polygon<4,T> {
	public:
		Polygon() { }
		Polygon(const toon::Vector<3,T>& v0,
						const toon::Vector<3,T>& v1,
						const toon::Vector<3,T>& v2,
						const toon::Vector<3,T>& v3) {
			verts[0] = v0;
			verts[1] = v1;
			verts[2] = v2;
			verts[3] = v3;
		}
		toon::Vector<3,T> verts[4];
	};

	// Represents an axis-aligned bounding box
	template <typename T/*=double*/>  // see polygon-fwd.h
	class Bounds2D {
	public:
		T left, right, top, bottom;
		Bounds2D() : left(0), right(0), top(0), bottom(0) { }
		Bounds2D(const T& l, const T& r, const T& t, const T& b)
			: left(l), right(r), top(t), bottom(b) { }
		// Get the coordinate of the corners
		toon::Vector<2,T> tl() const { return toon::makeVector(left, top); }
		toon::Vector<2,T> tr() const { return toon::makeVector(right, top); }
		toon::Vector<2,T> bl() const { return toon::makeVector(left, bottom); }
		toon::Vector<2,T> br() const { return toon::makeVector(right, bottom); }

		// Get the coordinate of the corners in homogeneous coordinates
		toon::Vector<3,T> htl() const { return toon::makeVector(left, top, 1); }
		toon::Vector<3,T> htr() const { return toon::makeVector(right, top, 1); }
		toon::Vector<3,T> hbl() const { return toon::makeVector(left, bottom, 1); }
		toon::Vector<3,T> hbr() const { return toon::makeVector(right, bottom, 1); }

		// Get homogeneous line equations for the boundaries. Interior of
		// this region is on the positive side of these lines.
		toon::Vector<3,T> left_eqn() const { return toon::makeVector(1,0,-left); }
		toon::Vector<3,T> right_eqn() const { return toon::makeVector(-1,0,right); }
		toon::Vector<3,T> top_eqn() const { return toon::makeVector(0,1,-top); }
		toon::Vector<3,T> bottom_eqn() const { return toon::makeVector(0,-1,bottom); }

		// Determine if the given point is inside these bounds
		template <typename U>
		bool Contains(const toon::Vector<2,U>& v) const {
			return v[0] >= left && v[0] <= right && v[1] >= top && v[1] <= bottom;
		}

		// Determine if the given point is inside these bounds
		template <typename U>
		bool Contains(const toon::Vector<3,U>& v) const {
			return (v[0] >= v[2]*left && v[0] <= v[2]*right &&
							v[1] >= v[2]*top && v[1] <= v[2]*bottom);
		}

		// Get a polygon representing the boundary of this region
		Polygon<4,T> GetPolygon() const {
			return Polygon<4,T>(htl(), htr(), hbr(), hbl());
		}

		// Construct bounds from any two opposing corners, in any order
		static Bounds2D<T> FromCorners(const toon::Vector<2>& a,
																	 const toon::Vector<2>& b) {
			return Bounds2D(min(a[0], b[0]), max(a[0], b[0]),
											min(a[1], b[1]), max(a[1], b[1]));
		}

		// Construct bounds from a size. Top-left corner is assumed to be at the origin.
		static Bounds2D<T> FromSize(const ImageRef& sz) {
			return Bounds2D(0, sz.x, 0, sz.y);
		}

		// Construct bounds from a size. Top-left corner is assumed to be at the origin.
		static Bounds2D<T> FromTightSize(const ImageRef& sz) {
			return Bounds2D(0, sz.x-1, 0, sz.y-1);
		}
	};
}
