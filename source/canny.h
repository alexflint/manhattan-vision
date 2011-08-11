#pragma once

#include <boost/shared_ptr.hpp>

#include "common_types.h"
#include "image_bundle.h"

namespace indoor_context {
using boost::shared_ptr;

// Compute pixelwise gradient magnitude and orientation
class Gradients {
public:
	// Algorithm parameters
	float atan_thresh;  // if the squared magnitude is below this
										  // threshold then the gradient orientation will
										  // not be computed

	// Last input image passed to Compute
	const ImageF* prev_input;

	// Outputs from the algorithm
	//ImageF smoothed;  // Gaussian smoothed image
	ImageF diffx, diffy;  // x- and y-sobel outputs
	MatF magnitude_sqr;  // Gradient magnitude squared
	MatF orient;  // Gradient orientation in radians
	MatI dir4;  // Gradient orientation binned 4 ways
	MatI dir16;  // Gradient orientation binned 16 ways

	// Initialize an empty gradient filter
	Gradients() : atan_thresh(0) { }
	// Initialize a gradient filter and run it for the specified image
	Gradients(const ImageF& input) : atan_thresh(0) {
		Compute(input);
	}
	Gradients(const ImageBundle& input) : atan_thresh(0) {
		Compute(input);
	}

	// Compute image derivatives, their magnitudes, and their orientations
	// i.e. all of the below
	void Compute(const ImageBundle& input);
	void Compute(const ImageF& input);

	// Fill diffx and diffy
	void ComputeSobel(const ImageF& input);

	// Fill magnitude_sqr
	void ComputeMagSqr();
	// Fill an interval of rows in magnitude_sqr
	void ComputeMagSqrRows(int r0, int r1);
	// Fill one row of magnitude_sqr matrix from diffx,diffy
	void ComputeMagSqrRow(int r);

	// Fill orient, dir4, and dir16
	void ComputeOrients();
	// Fill an interval of rows in orient, dir4, and dir16
	void ComputeOrientRows(int r0, int r1);
	// Fill one row of orient, dir4, and dir16 from magnitude_sqr
	void ComputeOrientRow(int r);
private:
	void ProcessRow(int r);
	void ProcessRows(int r0, int r1);
};


class CannyBase {
public:
	MatI edge_map;  // Binary edge map
	vector<ImageRef> edge_list;  // List of edge pixels

	// Suppress non-maximal edge responses
	void NonMaxSuppression(MatF& gradient_mag_sqr,
												 const MatI& gradient_dir4,
												 vector<ImageRef>& high_list);
	// Find points that are above the high threshold or else connected
	// to another edge point and above the low threshold.
	void Hysterisis(const MatF& gradient_mag_sqr,
									const vector<ImageRef>& high_list);
	// Run non-maximal suppression and hysterisis
	void DetectEdges(MatF& gradient_mag_sqr,
									 const MatI& gradient_dir4);
};


// Canny line detector, extended to multi-scale detection
class Canny : public CannyBase {
public:
	Gradients gradients;  // Gradient magnitude and orientation
	MatF& magnitude_sqr;  // ref to gradients.magnitude_sqr
	MatI& dir4;  // ref to gradients.dir4
	MatI& dir16;  // ref to gradients.dir16

	// Initialize an empty edge detector
	Canny()
	: magnitude_sqr(gradients.magnitude_sqr),
	  dir4(gradients.dir4),
	  dir16(gradients.dir16) { }
	// Initialize an edge detect and run it for the specified image
	Canny(ImageF& input)
	: magnitude_sqr(gradients.magnitude_sqr),
	  dir4(gradients.dir4),
	  dir16(gradients.dir16) {
		Compute(input);
	}

	// Execute the edge detector for this image
	void Compute(const ImageBundle& input);
	// Execute the edge detector for this image
	void Compute(const ImageF& input);

	// Visualize binary edge map
	void OutputEdgeViz(const string& path);
	// Visualize gradient magnitude
	void OutputMagnitudeViz(const string& path);
	// Visualize X sobel response MIGHT NOT WORK
	void OutputXSobelViz(const string& path);
	// Visualize Y sobel response MIGHT NOT WORK
	void OutputYSobelViz(const string& path);
};

// Multiscale canny line detector, extended to multi-scale detection
class MultiScaleCanny : public CannyBase {
public:
	// Gradient magnitude and orientation at each scale
	vector<shared_ptr<Gradients> > gradients;

	MatF magnitude_sqr;  // max magnitude over scales
	MatI dir4;  // ref to gradients[0]->dir4
	MatI dir16;  // ref to gradients[0]->dir16

	// Constructor
	MultiScaleCanny() : gradients(*gvar3<int>("MultiScaleCanny.NumScales")) {
		for (int i = 0; i < gradients.size(); i++) {
			gradients[i].reset(new Gradients);
		}
	}

	// Run the multi-scale detector for this image
	void Compute(const ImageF& input);
};

}
