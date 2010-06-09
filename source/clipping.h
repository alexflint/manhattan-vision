#pragma once

#include <limits>

#include "common_types.h"
#include "math_utils.tpp"

namespace indoor_context {
	// Return the sign of the dot product between two vectors
	inline int SignDP(const toon::Vector<>& a, const toon::Vector<>& b) {
		return Sign(a*b);
	}

	// Determines which side of a line a particular point lies on. Returns
	// -1, 0, or 1. For points at infinity that are not on the line,
	// returns -1 or 1 arbitrarily (since mathematically there is no
	// "true" result in this case.
	inline int HSignDP(const toon::Vector<>& a, const toon::Vector<>& b) {
		// Need to multiply by sign of second component because homogeneous
		// vectors can always be arbitrarily multiplied by some (potentially
		// negative) constant.
		return Sign(a*b) * HalfSign(a[2]) * HalfSign(b[2]);
	}

	// Returns true if a given line segment A crosses a given line . In
	// the special case that the start or end point of A is on B, return
	// false.
	inline bool SegmentCrosses(const toon::Vector<3>& start,
														 const toon::Vector<3>& end,
														 const toon::Vector<3>& line) {
		// Be careful because we are dealing with 9 possibilities (+,-,0)^2
		return HSignDP(start, line)*HSignDP(end, line) == -1;
	}

	// Returns true if a given line segment A crosses a given line . In
	// the special case that the start or end point of A is on B, return
	// true.
	inline bool SegmentTouches(const toon::Vector<3>& start,
														 const toon::Vector<3>& end,
														 const toon::Vector<3>& line) {
		// Be careful because we are dealing with 9 possibilities (+,-,0)^2
		return HSignDP(start, line)*HSignDP(end, line) != 1;
	};

	// Clip a line segment to the positive or negative side (specified by
	// side=1 or -1 respectively) of a line. Return false if the entire
	// line segment was outside the clip area, or true otherwise.
	bool ClipAgainstLine(toon::Vector<3>& start,
											 toon::Vector<3>& end,
											 const toon::Vector<3>& line,
											 const int side);

	// Fill the specified polygon in poly. Clips to the image bounds
	// automatically. Return the number of pixels changed.
	template <typename Range, typename Canvas, typename T>
	int FillPolygonFast(const Range& poly,
											Canvas& image,
											const T& v) {
		/*BOOST_STATIC_ASSERT((is_convertible<
												 typename range_value<Range>::type,
												 toon::Vector<3> >::value));*/
		if (poly.size() < 3) return 0;

		const int nx = Width(image);
		const int ny = Height(image);
		const int n = poly.size();

		int ymin = numeric_limits<int>::max(), ymax = numeric_limits<int>::min(), imin, imax;
		double ms[n], cs[n], ys[n];  // slopes, intercepts, y-coords
		bool ishoriz[n];
		for (int i = 0; i < n; i++) {
			const Vec3& u = poly[i];
			const Vec3& v = poly[(i+1)%n];

			ys[i] = u[1] / u[2];

			// check if two vertices are coincident
			if (norm(project(u)-project(v)) < 1e-9) {
				ishoriz[i] = true;
			} else {

				toon::Vector<3> line = u ^ v;
				ishoriz[i] = abs(line[0]) < 1e-6*abs(line[1]);
				if (!ishoriz[i]) {
					ms[i] = -line[1]/line[0];
					cs[i] = -line[2]/line[0];
				}

				if (ceili(ys[i]) < ymin) {
					ymin = ceili(ys[i]);
					imin = i;
				}
				if (floori(ys[i]) > ymax) {
					ymax = floori(ys[i]);
					imax = i;
				}
			}
		}

		int count = 0;
		int left = (imin+n-1)%n;  // ring decrement
		int right = imin;
		int ya = max(ymin, 0);
		int yb = min(ymax, ny);
		for (int y = ya; y < yb; y++) {
			while ((ys[left]<y || ishoriz[left]) && left != right) {
				left = (left+n-1)%n;  // ring decrement
			}
			while ((ys[(right+1)%n]<y || ishoriz[right]) && left != right) {
				right = (right+1)%n;  // ring increment
			}

			double xxa = ms[left]*y + cs[left];
			double xxb = ms[right]*y + cs[right];
			if (isnan(xxa) || isnan(xxb)) {
				DLOG << "Warning: NaN coordinates at y=" << y << " in FillPolygonFast";
				// ignore and move to next row
				continue;
			}

			int xa = Clamp(xxa, 0, nx-1);
			int xb = Clamp(xxb, 0, nx-1);

			fill(image[y]+min(xa,xb), image[y]+max(xa,xb), v);
			count += abs(xa-xb);
		}
		return count;
	}

	// Fill a convex polygon with a specified value
	template <typename Canvas, typename T>
	void FillPolygon(const vector<toon::Vector<3> >& poly,
									 Canvas& image,
									 const T& v) {
		const int nx = Width(image);
		const int ny = Height(image);
		const int n = poly.size();

		int ymin = ny-1, ymax = 0;
		vector<toon::Vector<3> > poly_bounds(poly.size());
		for (int i = 0; i < n; i++) {
			poly_bounds[i] = poly[i] ^ poly[(i+1)%n];
			int y = Clamp(roundi(poly[i][0] / poly[i][2]), 0, ny-1);
			if (y < ymin) ymin = y;
			if (y > ymax) ymax = y;
		}

		toon::Vector<3> scanline = toon::makeVector(0.0, 1.0, 0.0);
		for (int y = ymin; y <= ymax; y++) {
			scanline[2] = -y;
			// Find the end points
			int xmin = nx-1;
			int xmax = 0;
			for (int i = 0; i < poly.size(); i++) {
				if (SegmentTouches(poly[i], poly[(i+1)%n], scanline)) {
					toon::Vector<3> isct = scanline ^ poly_bounds[i];
					const int x = Clamp(roundi(isct[0] / isct[2]), 0, nx-1);
					if (x < xmin) xmin = x;
					if (x > xmax) xmax = x;
				}
			}

			// Draw the row (std::fill will segfault for xmin>xmax)
			if (xmin < xmax) {
				fill(image[y]+xmin, image[y]+xmax, v);
			}
		}
	}



	int PointSign(const toon::Vector<3>& point, const toon::Vector<3>& line);

	void ClipToPositive(const toon::Vector<3>& line,
											const toon::Vector<3>& clip,
											toon::Vector<3>& start,
											toon::Vector<3>& end);

	void GetImageBounds(const VW::ImageRef& size, vector<toon::Vector<3> >& bounds);

	void GetROIBounds(const VW::ROI& roi, vector<toon::Vector<3> >& bounds);

	// Take care here, poly must be a vector of line equations
	// representing the sides of the poly, NOT the coordinates of the
	// corners of the poly.
	void ClipLineToPoly(const toon::Vector<3>& line,
											const vector<toon::Vector<3> >& poly,
											toon::Vector<3>& out_a,
											toon::Vector<3>& out_b);

	void ClipLineToImage(const toon::Vector<3>& line,
											 const VW::ImageRef& size,
											 toon::Vector<3>& out_a,
											 toon::Vector<3>& out_b);

	void ClipLineToImage(const toon::Vector<3>& line,
											 const VW::ImageRef& size,
											 toon::Vector<2>& out_a,
											 toon::Vector<2>& out_b);

	void ClipLineToROI(const toon::Vector<3>& line,
										 const VW::ROI& roi,
										 toon::Vector<3>& out_a,
										 toon::Vector<3>& out_b);

	void ClipLineToROI(const toon::Vector<3>& line,
										 const VW::ROI& roi,
										 toon::Vector<2>& out_a,
										 toon::Vector<2>& out_b);
}
