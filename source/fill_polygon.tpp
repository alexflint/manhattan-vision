/*
 * fill_polygon.tpp
 *
 *  Created on: 6 Jul 2010
 *      Author: alexf
 */

#pragma once

#include "common_types.h"
#include "clipping.h"

#include "matrix_traits.tpp"
#include "vector_utils.tpp"

namespace indoor_context {

// Get a series of pairs [x0, x1] corresponding to horizontal strips in the image
// that are within the bounds of the given polygon. The first scanline corresponds to y0
// and then successive scanlines corresponding to successive rows.
// Return the number of pixels within the polygon boundaries.
template <typename Range>
int ComputeFillScanlines(const Range& poly,
                         const Vec2I& imsize,
                         int& y0,
                         vector<pair<int, int> >& scanlines) {
	const int n = poly.size();
	if (n < 3) return 0;

	int imin, imax;
	int ymin = numeric_limits<int>::max();
	int ymax = numeric_limits<int>::min();
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
			Vec3 line = u ^ v;
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

	y0 = max(ymin, 0);
	int yb = min(ymax, imsize[1]);
	for (int y = y0; y < yb; y++) {
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

		int xa = Clamp(xxa, 0, imsize[0]-1);
		int xb = Clamp(xxb, 0, imsize[0]-1);
		count += abs(xa-xb);
		scanlines.push_back(make_pair(min(xa,xb), max(xa,xb)));
	}
}

// Fill the specified polygon in poly. Clips the polygon to the image bounds
// before drawing. Return the number of pixels changed.
template <typename Range, typename Canvas, typename T>
int FillPolygon(const Range& poly, Canvas& canvas, const T& v) {
	int y0;
	vector<pair<int, int> > scanlines;
	Vec2I sz = matrix_size(canvas);
	int count = ComputeFillScanlines(poly, sz, y0, scanlines);

	for (int i = 0; i < scanlines.size(); i++) {
		fill(canvas[y0+i]+scanlines[i].first, canvas[y0+i]+scanlines[i].second+1, v);
	}
	return count;
}






// Fill a convex polygon with a specified value
template <typename Canvas, typename T>
void FillPolygonSlow(const vector<Vec3>& poly, Canvas& image, const T& v) {
	const int nx = matrix_width(image);
	const int ny = matrix_height(image);
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

}  // namespace indoor_context
