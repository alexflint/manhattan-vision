#pragma once

#include "common_types.h"
#include "vw_image-fwd.h"
#include "line_segment.h"

namespace indoor_context {
	class ManhattanReconstruction;
	class PosedCamera;
	class Canvas;
	class DPPayoffs;
	class DPGeometry;

	namespace proto {
	class FloorPlan;
	class TruthedFrame;
	}

	// Get the number of agreeing pixels
	int ComputeAgreement(const MatI& a, const MatI& b);
	// Get percentage of agreeing pixels
	double ComputeAgreementFrac(const MatI& a, const MatI& n);

	// Downsample the orientation map to the specified size
	void DownsampleOrients(const MatI& in, MatI& out, const Vec2I& size);
	// Downsample the orientation map by a factor k
	void DownsampleOrients(const MatI& in, MatI& out, int k);

	// Replace all instances of A in m with B, and vice versa
	// Should no longer be needed
	void InterchangeLabels(MatI& m, int a, int b);

	// Get the floor->ceiling homology
	Mat3 GetFloorCeilHomology(const PosedCamera& pc, const proto::FloorPlan& fp);

	// Convert a set of line segments to the path representation
	void SegmentsToPathUnclamped(const vector<LineSegment>& segments,
															 const vector<int>& segment_orients,
															 const DPGeometry& geometry,
															 VecI& path,
															 VecI& orients);

	// Draw an array of dots to visualize the given payoffs
	void DrawPayoffs(Canvas& canvas,
									 const DPPayoffs& payoffs,
									 const DPGeometry& geom);

	// Output a visualization of the given payoffs to the file specified
	void OutputPayoffsViz(const string& filename,
												const ImageRGB<byte>& orig,
												const DPPayoffs& payoffs,
												const DPGeometry& geom);

	// Compute per--pixel relative depth errors
	void ComputeDepthErrors(const MatD& estimaged_depth,
													const MatD& gt_depth,
													MatF& depth_errors);

	// Compute mean of a matrix of errors. Differs from generic mean
	// calculation only in that it checks that all errors are finite and
	// greater than zero.
	double MeanError(const MatF& errors);
}
