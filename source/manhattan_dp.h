#pragma once

#include <tr1/unordered_map>

#include <TooN/LU.h>

#include "common_types.h"
#include "camera.h"
#include "floor_ceil_map.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"
#include "map.pb.h"

#include "table.tpp"
#include "integral_col_image.tpp"
#include "histogram.tpp"

namespace indoor_context {

	// Downsample an orientation map to a specified resolution, taking a
	// majority vote in each cell
	void DownsampleOrients(const MatI& in, MatI& out, ImageRef res);

	// Downsample an orientation by a scaling factor k, taking a majority
	// vote in each cell
	void DownsampleOrients(const MatI& in, MatI& out, int k);



	// Represents a wall segment in an image
	struct ManhattanWall {
		Polygon<4> poly;
		int axis;
		inline ManhattanWall() { }
		inline ManhattanWall(const Vec3& tl,
		                     const Vec3& tr,
		                     const Vec3& br,
		                     const Vec3& bl,
		                     int a)
			: poly(tl, tr, br, bl), axis(a) {
		}
	};


	// Represents a node in the DP graph.
	// Be very careful about adding things to this class since
	// DPStateHasher reads this as an array of bytes. Don't add virtual
	// or elements that will lead to any uninitialized bytes for packing
	// reasons.
	struct DPState {
		enum { DIR_IN, DIR_OUT, DIR_UP, DIR_DOWN };
		short row, col, axis, remaining, dir;
		DPState();
		DPState(int r, int c, int a, int b, int d);
		static const DPState none;
		bool operator==(const DPState& other) const;
		bool operator!=(const DPState& other) const;
	};

	// Represents a hash function for DPState objects
	struct DPStateHasher {
		size_t operator()(const DPState & dpstate) const;
	};

	// Represents the solution at a node in the DP problem
	struct DPSolution {
		double score;
		DPState src;
		DPSolution();
		DPSolution(double s);
		DPSolution(double s, const DPState& state);
		void ReplaceIfSuperior(const DPSolution& other,
		                       const DPState& state,
		                       double delta=0);
	};

	// Output operators
	ostream& operator<<(ostream& s, const DPState& x);
	ostream& operator<<(ostream& s, const DPSolution& x);

	// Represents a cache of DP states and their solutions
	class DPCache {
	public:
		typedef DPSolution* iterator;
		Table<5, DPSolution> table;
		void reset(const Vec2I& grid_size, int max_corners);
		void clear();
		iterator begin();
		iterator end();
		iterator find(const DPState& state);
		DPSolution& operator[](const DPState& state);
	};



	class DPCost {
	public:
		virtual Vec2I GetGridSize() const;
		virtual int GetWallCost(int x, int y0, int y1, int orient) const = 0;
		virtual int GetCornerCost(int x, int y0, iny y1) const = 0;
	};


	// Find optimal indoor Manhattan structures by dynamic programming
	class ManhattanDP {
	public:
		typedef	DPCache Cache;

		const MatI* input_orients;

		// Input parameters: these are initialized to the respective GVar values but can be
		// modified programatically before Compute()
		double jump_thresh;
		int vert_axis;
		int max_corners;
		int corner_penalty;
		int grid_scale_factor;
		double grid_offset_factor;

		// output values
		MatI orient_map;  // noisy pixel-wise orientation estimate
		IntegralColImage<3> integ_orients;

		Vec2I input_size, grid_size;  // size of input_orients and est_orients respective
		Mat3 imageToGrid, gridToImage;  // mapping from image to rectified grid and back
		Mat3 floorToCeil;  // planar homology from floor to ceiling in image coordinates
		MatI opp_rows;  // cache of opposite (floor<->ceil) rows as passed through floorToCeil
		//FloorCeilMap fcmap;  // deprecated mapping from floor to ceiling. TODO: remove this

		// These are all deprecated...
		/*int grid_scaling;  // actually the reciprocal of the scaling
		Vec2I grid_offset;  // the extra padding in the orientation map
		Mat3 H_canon;  // homography to make vertical lines vertical in the image
		Mat3 H_canon_inv;  // inverse of above, cached for efficiency
		*/

		const PosedCamera* pc;
		Vec2 horiz_vpts[2];
		int horiz_vpt_cols[2];
		int horizon_row;

		// The cache of DP evaluations
		Cache cache;

		// The man solution parameters
		DPSolution solution;  // the solution node
		vector<const DPState*> full_backtrack;  // series of nodes to the solution
		vector<const DPState*> abbrev_backtrack;  // as above but omitting UP, DOWN, and some OUT nodes
		vector<ManhattanWall> soln_walls; // series of quads representing the solution in image coords
		vector<ManhattanWall> soln_grid_walls; // series of quads representing the solution in grid coords
		MatI soln_orients;  // orientation map as predicted by the optimal model

		// Performance statistics
		double solve_time;
		int cache_lookups, cache_hits, max_depth, cur_depth;
		LazyHistogram<int> horiz_len_hist;
		LazyHistogram<int> horiz_cutoff_hist;

		// Default constructor.
		ManhattanDP();

		// Compute the optimal manhattan model (does all of the below).
		void Compute(const MatI& orients, const PosedCamera& cam, const Mat3& floorToCeil);
		// Populate H_canon and H_canon_inv with appropriate warps
		void ComputeGridWarp();
		// Populate integ_orients with warped orientation data
		void ComputeOrients();
		// Populate opp_rows according to fcmap
		void ComputeOppositeRows();
		// Backtrack through the evaluation graph from the solution
		void ComputeBacktrack();

		// Convert between image and grid coordinates
		Vec3 GridToImage(const Vec2& x);
		Vec2 ImageToGrid(const Vec3& x);

		// The caching wrapper for the DP
		const DPSolution& Solve(const DPState& state);
		// The DP implementation
		DPSolution Solve_Impl(const DPState& state);

		// Determines whether an occlusion is physically realisable
		// occl_side should be -1 for left or 1 for right
		bool OcclusionValid(int col, int left_axis, int right_axis, int occl_side);
		// Determine whether occlusion constraints allow us to move
		// between two DP nodes.
		bool CanMoveVert(const DPState& cur, const DPState& next);
		
		// Draw a series of walls as quads with crosses through them
		void DrawWalls(ImageRGB<byte>& canvas, 
		               const vector<ManhattanWall>& walls) const;

		// Draw the best solution
		void DrawSolution(ImageRGB<byte>& canvas) const;
		// Draw the best solution the original grid frame
		void DrawGridSolution(ImageRGB<byte>& canvas) const;
	};



	class ManhattanDPReconstructor {
	public:
		const PosedImage* input;
		ManhattanDP dp;
		GuidedLineDetector line_detector;
		IsctGeomLabeller line_sweeper;

		void Compute(const PosedImage& pim, const proto::TruthedMap& tru_map);
		void ReportBacktrack();

		// Compute accuracy w.r.t. ground truth
		double GetAccuracy(const MatI& gt_orients);
		// Compute accuracy w.r.t. the ground truth floorplan
		double GetAccuracy(const proto::FloorPlan& gt_floorplan);

		// Draw the original image
		void OutputOrigViz(const string& path);
		// Draw lines and line sweeps
		void OutputOrientViz(const string& path);
		// Draw the line detections
		void OutputLineViz(const string& path);
		// Draw the solution
		void OutputSolutionOrients(const string& path);
		// Draw the solution in grid coordinates
		void OutputGridViz(const string& path);
		// Vizualize the opp_rows matrix
		void OutputOppRowViz(const string& path);
		// Transfer the solution to an auxilliary view
		//void GetAuxOrients(const PosedCamera& aux, double zfloor, MatI& aux_orients);
		// Draw the solution in an auxilliary view
		/*void OutputSolutionInView(const string& path,
		                          const Frame& aux,
		                          double zfloor);*/
	};
}
