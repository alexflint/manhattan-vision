#include "clipping.h"
#include "common_types.h"

namespace indoor_context {
using namespace toon;
using VW::ROI;
using VW::ImageRef;

bool ClipAgainstLine(Vector<3>& start,
                     Vector<3>& end,
                     const Vector<3>& line,
                     const int side) {
	bool clipped_start = false;
	bool clipped_end = false;
	if (HSignDP(start, line) != side) {
		start = line ^ (start ^ end);
		clipped_start = true;
	}
	if (HSignDP(end, line) != side) {
		end = line ^ (start ^ end);
		clipped_end = true;
	}
	return !(clipped_start && clipped_end);
}

int PointSign(const Vector<3>& point, const Vector<3>& line) {
	return Sign(line*point) * HalfSign(point[2]);
}

void ClipToPositive(const Vector<3>& line,
                    const Vector<3>& clip,
                    Vector<3>& start,
                    Vector<3>& end) {
	const Vector<3> isct = line^clip;
	if (PointSign(line, start) == -1) {
		start = isct;
	}
	if (PointSign(line, end) == -1) {
		end = isct;
	}
}

void GetImageBounds(const ImageRef& size,
                    vector<Vector<3> >& bounds) {
	bounds.push_back(makeVector(1.0, 0.0, 0.0)); // left
	bounds.push_back(makeVector(0.0, 1.0, 0.0)); // top
	bounds.push_back(makeVector(-1.0, 0.0, size.x-1)); // right
	bounds.push_back(makeVector(0.0, -1.0, size.y-1)); // bottom
}

void GetROIBounds(const ROI& roi,
                  vector<Vector<3> >& bounds) {
	bounds.push_back(makeVector(1.0, 0.0, -roi.Left())); // left
	bounds.push_back(makeVector(0.0, 1.0, -roi.Top())); // top
	bounds.push_back(makeVector(-1.0, 0.0, roi.Right()-1)); // right
	bounds.push_back(makeVector(0.0, -1.0, roi.Bottom()-1)); // bottom
}

void ClipLineToPoly(const Vector<3>& line,
                    const vector<Vector<3> >& poly,
                    Vector<3>& a,
                    Vector<3>& b) {
	a = Zeros;
	b = Zeros;
	const int n = poly.size();
	INDENTED for (int i = 1; i <= n; i++) {
		const Vector<3> isct = line^poly[i%n];
		const float dp1 = PointSign(isct, poly[ (i-1)%n ]);
		const float dp2 = PointSign(isct, poly[ (i+1)%n ]);
		if (dp1 >= 0 && dp2 >= 0) {
			b = a;
			a = isct;
		}
	}
}


void ClipLineToImage(const Vector<3>& line,
                     const ImageRef& size,
                     Vector<3>& a,
                     Vector<3>& b) {
	vector<Vector<3> > bounds;
	GetImageBounds(size, bounds);
	ClipLineToPoly(line, bounds, a, b);
}

void ClipLineToImage(const Vector<3>& line,
                     const ImageRef& size,
                     Vector<2>& a,
                     Vector<2>& b) {
	Vector<3> ah, bh;
	ClipLineToImage(line, size, ah, bh);
	a = project(ah);
	b = project(bh);
}

void ClipLineToROI(const Vector<3>& line,
                   const ROI& roi,
                   Vector<3>& a,
                   Vector<3>& b) {
	vector<Vector<3> > bounds;
	GetROIBounds(roi, bounds);
	ClipLineToPoly(line, bounds, a, b);
}

void ClipLineToROI(const Vector<3>& line,
                   const ROI& roi,
                   Vector<2>& a,
                   Vector<2>& b) {
	Vector<3> ah, bh;
	ClipLineToROI(line, roi, ah, bh);
	a = project(ah);
	b = project(bh);
}
}
