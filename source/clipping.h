#pragma once

#include <limits>

#include "common_types.h"
#include "math_utils.tpp"
#include "image_utils.tpp"

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
