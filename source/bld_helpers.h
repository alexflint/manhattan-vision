#pragma once

#include "common_types.h"
#include "vw_image-fwd.h"

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

	// Get the path to the truthed_map.pro file for a sequence, or die
	// if the sequence is not found.
	string GetMapPath(const string& sequence_name);

	// Get the number of agreeing pixels
	int ComputeAgreement(const MatI& a, const MatI& b);
	// Get percentage of agreeing pixels
	double ComputeAgreementPct(const MatI& a, const MatI& n);

	// Downsample the orientation map to the specified size
	void DownsampleOrients(const MatI& in, MatI& out, const Vec2I& size);
	// Downsample the orientation map by a factor k
	void DownsampleOrients(const MatI& in, MatI& out, int k);

	// Replace all instances of A in m with B, and vice versa
	// Should no longer be needed
	void InterchangeLabels(MatI& m, int a, int b);

	// Get the floor->ceiling homology
	Mat3 GetFloorCeilHomology(const PosedCamera& pc, const proto::FloorPlan& fp);

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

	// Compute per--pixel relative depth errors
	double ComputeMeanDepthError(const MatF& errors);
}
