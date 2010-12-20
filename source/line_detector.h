#pragma once

#include <queue>

#include "image_bundle.h"
#include "common_types.h"
#include "canny.h"
#include "line_segment.h"

namespace indoor_context {

// Output operator for line segments
//ostream& operator<< (ostream& s, const LineSeg& seg);

class LineDetection {
public:
	LineDetection()
	: eqn(toon::Zeros),
	  confidence(0),
	  axis(-1),
	  pixels(new vector<ImageRef>) { }
	LineDetection(const Vec3& a, const Vec3& b, int ax=-1)
	: seg(a,b),
	  eqn(a^b),
	  confidence(0),
	  axis(ax),
	  pixels(new vector<ImageRef>) {
	}
	LineSeg seg;
	Vec3 eqn;
	double confidence;
	int axis;
	shared_ptr<vector<ImageRef> > pixels;

	void DrawPixels(ImageRGB<byte>& canvas,
	                const PixelRGB<byte>& color,
	                Vec2 offset=toon::Zeros,
	                int thickness=1) const;
};

// Finds line segments using the algorithm of (Kosecka and Zhang, 2002)
class CannyLineDetector {
public:
	// Last image passed to Compute()
	const ImageBundle* input;
	// Line segments found in the image
	vector<LineDetection> detections;
	// Map of pixel to the line segment that contains them. Pixels not
	// part of any line segment are set to -1
	MatI segment_map;
	// The canny edge detector. Included here to allow it to re-use its
	// buffers across several invokations.
	Canny canny;

	// Initializes the line detector empty
	CannyLineDetector();
	// Initializes the line detector and execute it for the given image
	CannyLineDetector(const ImageBundle& input);
	// Run the detector
	void Compute(const ImageBundle& input);

	// Draw line segments
	void Draw(ImageRGB<byte>& canvas) const;
	// Draw line segments offset from their original location
	void Draw(ImageRGB<byte>& canvas,
	          const VecI& grouping,
	          const toon::Vector<2> offs=toon::Zeros) const;
	// Draw line segments with specified colors
	void Draw(ImageRGB<byte>& canvas,
	          const vector<PixelRGB<byte> > colors,
	          const toon::Vector<2> offs=toon::Zeros) const;
	// Output a vizualization of line segments
	void OutputLineViz(const string& filename);
private:
	// Find connected components in the current edge map
	void FindComponents();
	// Fit lines to each of the current connected components
	void FitLines();

	// Internal buffers included here only to allow memory re-use
	MatI seen;
	queue<ImageRef> comp_queue;
};
}
