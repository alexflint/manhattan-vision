#pragma once

#include "common_types.h"

#include "floorplan_renderer.h"

namespace indoor_context {
	namespace proto { class FloorPlan; }
	class PosedCamera;

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

		// Construct empty
		ManhattanGroundTruth();
		// Construct and compute
		ManhattanGroundTruth(const proto::FloorPlan& gt_floorplan,
												 const PosedCamera& camera);
		// Compute from a floorplan and camera
		void Compute(const proto::FloorPlan& gt_floorplan,
								 const PosedCamera& camera);
		// Count walls and occlusions (called automatically by Compute)
		void CountWalls(const proto::FloorPlan& gt_floorplan,
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

		// Visualizations
		void DrawOrientations(ImageRGB<byte>& canvas, float alpha=1.0);
		void DrawDepthMap(ImageRGB<byte>& canvas);
		void OutputOrientations(const ImageRGB<byte>& bg,
														const string& filename);
		void OutputDepthMap(const string& filename);
	};
}
