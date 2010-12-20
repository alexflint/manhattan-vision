#pragma once

#include <queue>

#include <boost/ptr_container/ptr_vector.hpp>

#include "image_bundle.h"
#include "common_types.h"
#include "canny.h"
#include "rotation_estimator.h"
#include "line_detector.h"
#include "camera.h"

namespace indoor_context {
// Finds vanishing points using EM
class ManhattanFrameEstimator {
public:
	//
	// Inputs
	//

	// The line detections
	vector<LineDetection>* detections;
	// The equations of the above lines, cached in this form for efficiency
	vector<Vec3 > line_eqns;

	//
	// Outputs
	//

	// Best rotation
	toon::SO3<> R;
	// The vanishing points (just the cols of R)
	Vec3 vpts[3];
	// Number of line segments supporting each vanishing point
	int support[3];
	// Number of line segments labelled as spurious
	int num_spurious;

	// The K-means cluster index that each line segment belongs to
	VecI kmeans_owners;

	//
	// These are updated during the estimation process:
	//

	// Current log likelihood
	double loglik;
	// Number of iterations so far
	int num_iters;
	// Converged?
	bool converged;
	// Responsibility of each vanishing point for each line segment.
	MatD resps;
	// Rotation estimator
	RotationEstimator rot_est;

	// Compute vanishing points
	void Compute(vector<LineDetection>& segments, bool use_prev = false);

	//
	// Detailed control over EM
	//

	// Resize internal buffers in preparation for new observations
	void Prepare(const vector<LineDetection>& segments);
	// Bootstrap the EM process by running K-means
	void Bootstrap(const vector<LineDetection>& segments);
	// Compute responsibilities given current vanishing points
	void EStep();
	// Compute vanishing points given current responsibilities.
	// Also check for convergence.
	void MStep();
	// Populate owners, num_spurious
	void Finish();

	// Evaluate the likelihood of a line segment arising from a given
	// vanishing point.
	static double GetLogLik(const Vec3& vpt,
	                        const Vec3& line);
	// Fit the least-squares intersection point using SVD
	Vec3 FitIsct(const vector<LineDetection>& segments,
	             const toon::Vector<-1>& weights = toon::Zeros) const;
	// Fit a robust intersection point using RANSAC
	Vec3 FitIsctRansac(const vector<LineDetection>& segments,
	                   const toon::Vector<-1>& weights = toon::Zeros) const;

	// Draw the line detections and vanishing points
	void DrawVptViz(ImageRGB<byte>& canvas,
	                const ATANCamera& cam,
	                const ImageBundle& orig) const;
};

// Common base functionality for vanishing point detectors
class VanishingPoints {
public:
	// The long straight line detector
	CannyLineDetector lines;
	// The vanishing point detector
	ManhattanFrameEstimator manhattan_est;
	// The last input image
	const CalibratedImage* input;
	// The vanishing points in the image
	Vec3 image_vpts[3];

	// Find vanishing points. If use_prev is true then the EM will be
	// initialized with the result of the last iteration
	void Compute(const CalibratedImage& image, bool use_prev = false);

	// Output a visualization of the final vanishing points
	void DrawVanPointViz(ImageRGB<byte>& canvas) const;
	// Output a visualization of the final vanishing points with
	// specified vanishing point colors
	void DrawVanPointViz(ImageRGB<byte>& canvas,
	                     vector<PixelRGB<byte> > pallete) const;

	// Output a visualization of the final vanishing points
	void OutputVanPointViz(const string& filename) const;
	// Output the segments found in the image
	void OutputLineViz(const string& filename) const;
	// Draw line segments with two vanishing points highlighted
	void DrawHighlightViz(ImageRGB<byte>& canvas, const VecI& indices);
	// Draw line segments with two vanishing points highlighted
	void OutputHighlightViz(const string& filename, const VecI& indices);
};
}
