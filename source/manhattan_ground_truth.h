#pragma once

#include "common_types.h"
#include "line_segment.h"

#include "floorplan_renderer.h"

namespace indoor_context {
	namespace proto { class FloorPlan; }
	class PosedCamera;
	class DPGeometry;
	class DPGeometryWithScale;

	// Compute ground truth orientations in a frame from a ground truth
	// floorplan and camera. Also counts the number of walls and occlusions.
	class ManhattanGroundTruth {
	public:
		// The floorplan provided to Compute()
		const proto::FloorPlan* floorplan;
		// The camera provided to Compute()
		const PosedCamera* camera;

		// The renderer used to draw orientations and compute depths
		FloorPlanRenderer renderer;
		// Total number of walls (including occluding corners etc)
		int nwalls;
		// Total number of occluding corners (always less than nwalls)
		int nocclusions;
		// Vertices of the floor/wall boundary in image coordinates, as
		// start,end pairs.
		vector<LineSegment> floor_segments;
		vector<LineSegment> ceil_segments;
		// Orientation of the vertices above
		vector<int> segment_orients;

		// Construct empty
		ManhattanGroundTruth();
		// Construct and compute
		ManhattanGroundTruth(const proto::FloorPlan& gt_floorplan,
												 const PosedCamera& camera);
		// Compute from a floorplan and camera
		void Compute(const proto::FloorPlan& gt_floorplan,
								 const PosedCamera& camera);
		// Count walls and occlusions (called automatically by Compute)
		void ProcessWalls(const proto::FloorPlan& gt_floorplan,
											const PosedCamera& camera);

		// Get an image with labels drawn from {0,1,2} 
		const MatI& orientations() const { return renderer.orientations(); }
		// Get a depth image
		const MatD& depthmap() const { return renderer.depthmap(); }
		// Get the number of walls (includes occluding walls too)
		int num_walls() const { return nwalls; }
		// Get the number of occlusions (always less than num_walls)
		int num_occlusions() const { return nocclusions; }
		// Get the ground plane z positions
		double zfloor() const;
		// Get the ceiling plane z positions
		double zceil() const;

		// Compute the floor and ceiling paths
		void ComputePathPairUnclamped(const DPGeometry& geometry,
																	VecI& ceil_path,
																	VecI& floor_path) const;

		// Compute path+orients representation of the solution
		// Return value is in grid coordinates
		// Orients are labelled according to opposite vpt (like orientations)
		void ComputePathAndOrients(const DPGeometry& geometry,
															 VecI& path,
															 VecI& orients) const;

		// As above but axes are labelled according to associated vpt
		void ComputePathAndAxes(const DPGeometry& geometry,
														VecI& path,
														VecI& axes) const;

		// Compute loss terms using L1 distance
		// Returned losses are in grid coordinates
		void ComputeL1LossTerms(const DPGeometry& geometry,
														MatF& loss_terms) const;

		// Compute loss terms for labelling error. Correctiosn are applied
		// so that losses correspond to error in image coordinates, though
		// the losses are returned in grid coordinates. That is, if you
		// evaluate a particular hypothesis on this loss matrix then the
		// outcome will equal the loss if you had compared that hypothesis
		// in image coordinates with the ground truth, also in image
		// coordinates.
		void ComputeLabellingLossTerms(const DPGeometry& geometry,
																	 MatF& loss_terms) const;

		// Compute loss terms for mean relative reprojection error
		// Returned losses are in grid coordinates
		void ComputeDepthLossTerms(const DPGeometryWithScale& geometry,
															 MatF& loss_terms) const;

		// Visualizations
		void DrawOrientations(ImageRGB<byte>& canvas, float alpha=1.0) const;
		void DrawDepthMap(ImageRGB<byte>& canvas) const;
		void OutputOrientations(const ImageRGB<byte>& bg,
														const string& filename) const;
		void OutputDepthMap(const string& filename) const;
	};
}
