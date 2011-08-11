#pragma once

#include <boost/array.hpp>

#include "common_types.h"
#include "camera.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"
#include "manhattan_ground_truth.h"
#include "simple_renderer.h"

#include "table.tpp"
#include "histogram.tpp"

namespace indoor_context {
	// From dp_payoffs.h
	class DPPayoffs;
	class DPObjective;

	extern "C" lazyvar<Vec2> gvGridSize;
	extern "C" lazyvar<float> gvLineJumpThreshold;
	
	////////////////////////////////////////////////////////////////////////////////
	// Represents a node in the DP graph.
	// Be very careful about adding things to this class since
	// DPStateHasher reads this as an array of bytes. Don't add virtual
	// or elements that will lead to any uninitialized bytes for packing
	// reasons.
	struct DPState {
		enum { DIR_IN, DIR_OUT, DIR_UP, DIR_DOWN };
		short row, col, axis, dir;
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

	////////////////////////////////////////////////////////////////////////////////
	// Represents the solution to a sub-problem in the DP problem
	// mostly used internally to cache and lookup solutions
	struct DPSubSolution {
		double score;
		DPState src;
		DPSubSolution();
		DPSubSolution(double s);
		DPSubSolution(double s, const DPState& state);
		bool ReplaceIfSuperior(const DPSubSolution& other,
													 const DPState& state,
													 double delta=0);
	};

	// Output operators
	ostream& operator<<(ostream& s, const DPState& x);
	ostream& operator<<(ostream& s, const DPSubSolution& x);

	////////////////////////////////////////////////////////////////////////////////
	// Represents a cache of DP states and their solutions
	class DPCache {
	public:
		typedef DPSubSolution* iterator;
		Table<4, DPSubSolution> table;
		void reset(const Vec2I& grid_size);
		void clear();
		inline iterator begin() { return table.begin(); }
		inline iterator end() { return table.end(); }
		iterator find(const DPState& state);
		DPSubSolution& operator[](const DPState& state);
	};

	////////////////////////////////////////////////////////////////////////////////
	// Represents a solution to an entire DP problem
	// Unlike DPSubSolution, this is mostly used externally to examine the
	// solution produced by ManhattanDP.
	class DPGeometryWithScale;
	class DPSolution {
	public:
		// The vector of Y values for each column. Size equals width of the payoff matrix
		VecI path_ys;
		// The vector or orientations for each column.
		VecI path_axes;
		// The number of walls in the solution
		int num_walls;
		// The number of occlusions in the solution
		int num_occlusions;
		// The score computed by ManhattanDP
		double score;
		// Orientation of each pixel predicted by this solution, in image coordinates
		MatI pixel_orients;

		// Line segments representing either the top or the bottom of walls, in image coords
		vector<LineSeg> wall_segments;
		// Orientations of the above wall segments
		vector<int> wall_orients;

		// Used in ComputeDepthMap
		SimpleRenderer renderer;

		// The node in the DP cache that corresponds to this solution
		DPSubSolution node;

		// Compute the value for a hypothesized solution (this is used for
		// reporting only, not by ManhattanDP). If the second argument is
		// false then ignore wall and occlusion penalties.
		double GetTotalPayoff(const DPPayoffs& payoffs,
													bool subtract_penalties=true) const;
		// Sum over the path represented by this solution through the
		// payoff matrix. Ignores axes -- see above for that.
		double GetPathSum(const MatF& payoffs) const;
		// Compute depth map for the solution given floor and ceiling heights
		// This returns a reference to an internal buffer.
		const MatD& GetDepthMap(const DPGeometryWithScale& geometry);
	};

	////////////////////////////////////////////////////////////////////////////////
	// Represents a transformation from image coordinates to a rectified grid
	class DPGeometry {
	public:
		const PosedCamera* camera; // the camera extrinsics
		Mat3 floorToCeil; // floor to ceiling mapping in image coordinates
		Vec2I grid_size;  // size of input_orients and est_orients respective

		Mat3 imageToGrid, gridToImage;  // mapping from image to rectified grid and back
		int horizon_row, vpt_cols[3];  // the horizon and vanishing points in grid coordinates
		Mat3 grid_floorToCeil, grid_ceilToFloor; // floor/ceiling mapping in grid coordinates

		// Initializes grid_size to the gvar value. Can be modified before Configure()
		DPGeometry();
		// Initialize and configure
		DPGeometry(const PosedCamera& camera, const Mat3& floorToCeil);

		// Accessors
		int nx() const { return grid_size[0]; }
		int ny() const { return grid_size[1]; }

		// Configure the various homographies and useful vanishing point info
		void Configure(const PosedCamera& camera, const Mat3& floorToCeil);

		// Convert between image and grid coordinates
		Vec3 GridToImage(const Vec2& x) const;
		Vec2 ImageToGrid(const Vec3& x) const;

		// Transform data from image to grid coordinates. Each element of
		// OUT is a sum over pixels from IN that project there according to
		// ImageToGrid(). No normalization is applied.
		void TransformDataToGrid(const MatF& in, MatF& out) const;
		// Transform image to grid coordinates
		void TransformToGrid(const ImageRGB<byte>& in,
												 ImageRGB<byte>& out) const;

		// Transfer a point between the floor and ceiling (is always self-inverting)
		Vec2 Transfer(const Vec2& grid_pt) const;
		Vec3 Transfer(const Vec3& grid_pt) const;

		// Get the top and bottom of the wall corresponding to a given grid point
		void GetWallExtent(const Vec2& grid_pt, int axis, int& y0, int& y1) const;
		// Transform a path to an orientation map in the grid domain
		void PathToOrients(const VecI& path, const VecI& path_axes, MatI& grid_orients) const;
	};

	////////////////////////////////////////////////////////////////////////////////
	// Represents an "upgraded" type of geometry with 3D scale information
	class DPGeometryWithScale : public DPGeometry {
	public:
		double zfloor, zceil;
		// Initializes grid_size to the gvar value. Can be modified before Configure()
		DPGeometryWithScale();
		// Initialize and configure
		DPGeometryWithScale(const PosedCamera& camera, double zfloor, double zceil);
		// Initialize and configure
		DPGeometryWithScale(const DPGeometry& geom, double zfloor, double zceil);
		// Configure the various homographies and useful vanishing point info
		void Configure(const PosedCamera& camera, double zfloor, double zceil);
		// Configure the various homographies and useful vanishing point info
		void Configure(const DPGeometry& geom, double zfloor, double zceil);
		// Back-project an image point onto the floor or ceiling plane
		Vec3 BackProject(const Vec3& image_point) const;
	};



	////////////////////////////////////////////////////////////////////////////////
	// Find optimal indoor Manhattan structures by dynamic programming
	class ManhattanDP {
	public:
		// Scalar parameters
		// these are initialized to the respective GVar values but can be
		// modified programatically before Compute()
		double jump_thresh;
		int vert_axis;

		// The function to optimize
		const DPPayoffs* payoffs;
		// Geometry info
		const DPGeometry* geom; // an input parameter
		//MatI opp_rows;  // cache of floor<->ceil mapping as passed through floorToCeil

		// The cache of DP evaluations
		DPCache cache;

		// The solution. TODO: clean up some of the items below
		DPSolution solution;

		// The solution and derived quantities
		vector<const DPState*> full_backtrack;  // series of nodes to the solution
		vector<const DPState*> abbrev_backtrack;  // as above but omitting UP, DOWN, and some OUT nodes

		// Performance statistics
		double solve_time;
		int cache_lookups, cache_hits, max_depth, cur_depth;
		LazyHistogram<int> horiz_len_hist;
		LazyHistogram<int> horiz_cutoff_hist;

		// Initializes input parameters from GVar values.
		ManhattanDP();

		// Compute the optimal manhattan model from the per-node score
		// matrix, which expresses the problem in terms of marginal costs of
		// building walls of each orientation at each pixel.
		void Compute(const DPPayoffs& payoffs,
								 const DPGeometry& geometry);
		// Backtrack through the evaluation graph from the solution
		void PopulateSolution(const DPSubSolution& soln_node);

		// Get the exact pixel-wise orientations for the current solution,
		// in grid coordinates. Uses GetSolutionPath etc.
		void ComputeGridOrients(MatI& grid_orients);
		// Get the exact pixel-wise orientations, in image coordinates, for
		// the current solution. This approach is slow because we project
		// each pixel individually into grid coordates to determine its
		// orientation.
		void ComputeExactOrients(MatI& orients);

		// The caching wrapper for the DP
		const DPSubSolution& Solve(const DPState& state);
		// The DP implementation
		DPSubSolution Solve_Impl(const DPState& state);
		// Determines whether an occlusion is physically realisable
		// occl_side should be -1 for left or 1 for right
		bool OcclusionValid(int col, int left_axis, int right_axis, int occl_side);
		// Determine whether occlusion constraints allow us to move
		// between two DP nodes.
		bool CanMoveVert(const DPState& cur, const DPState& next);

		// Draw wireframe walls in image coordinates
		void DrawWireframeSolution(ImageRGB<byte>& canvas) const;
		// Draw wireframe walls in grid coordinates
		void DrawWireframeGridSolution(ImageRGB<byte>& canvas) const;
	};




	////////////////////////////////////////////////////////////////////////////////
	// Wraps ManhattanDP, providing visualization routines that utilize a provided image
	class MonocularPayoffGen;

	class ManhattanDPReconstructor {
	public:
		const PosedImage* input;
		const DPPayoffs* payoffs;  // might point to this->payoff_gen->payoffs, or an external object
		ManhattanDP dp;
		DPGeometry geometry;

		// Might not be initialized, depending on which Compute() is called...
		// This is a pointer to avoid circular dependencies
		scoped_ptr<MonocularPayoffGen> payoff_gen;

		// Empty constructor (for scoped_ptr to incomplete type, ugh)
		ManhattanDPReconstructor();
		// Empty destructor (for scoped_ptr to incomplete type, ugh)
		~ManhattanDPReconstructor();
		// Compute the reconstruction for the given objective
		void Compute(const PosedImage& image,
								 const DPGeometry& geometry,
								 const DPObjective& objective);
		// Compute the reconstruction for the given payoff function
		void Compute(const PosedImage& image,
								 const DPGeometry& geometry,
								 const DPPayoffs& payoffs);

		// Report the solution as a sequence of DP nodes
		void ReportBacktrack();

		// Compute pixel--wise labelling accuracy w.r.t. ground truth
		double GetAccuracy(const MatI& gt_orients);
		// Compute pixel--wise labelling accuracy w.r.t. the ground truth floorplan
		double GetAccuracy(const ManhattanGroundTruth& gt);
		// Report and return classification accuracy.
		double ReportAccuracy(const ManhattanGroundTruth& gt);

		// Compute per-pixel relative-depth-error
		void GetDepthErrors(const ManhattanGroundTruth& gt,
												MatF& out_errors);
		// Compute mean relative-depth-error
		double GetDepthError(const ManhattanGroundTruth& gt);
		// Report and return mean relative-depth-error
		double ReportDepthError(const ManhattanGroundTruth& gt);

		// Draw the original image
		void OutputOrigViz(const string& path);
		// Output the solution orientations overlayed on the input image
		void OutputSolution(const string& path);
		// Draw the solution in grid coordinates
		void OutputGridViz(const string& path);
		// Draw a visualization of the floor-to-ceiling mapping
		void OutputManhattanHomologyViz(const string& path);
		// Draw a visualization of the payoffs with the solution model superimposed
		void OutputPayoffsViz(int orient, const string& path);
		// Draw a visualization of the relative depth error with respect to ground truth
		void OutputDepthErrorViz(const ManhattanGroundTruth& gt,
														 const string& path);
	};

}
