#pragma once

#include <boost/array.hpp>

#include "common_types.h"
#include "camera.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"
#include "map.pb.h"
#include "simple_renderer.h"

#include "table.tpp"
#include "integral_col_image.tpp"
#include "histogram.tpp"

namespace indoor_context {

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
// Represents the solution at a node in the DP problem
struct DPSolution {
	double score;
	DPState src;
	DPSolution();
	DPSolution(double s);
	DPSolution(double s, const DPState& state);
	bool ReplaceIfSuperior(const DPSolution& other,
	                       const DPState& state,
	                       double delta=0);
};

// Output operators
ostream& operator<<(ostream& s, const DPState& x);
ostream& operator<<(ostream& s, const DPSolution& x);

////////////////////////////////////////////////////////////////////////////////
// Represents a cache of DP states and their solutions
class DPCache {
public:
	typedef DPSolution* iterator;
	Table<4, DPSolution> table;
	void reset(const Vec2I& grid_size);
	void clear();
	inline iterator begin() { return table.begin(); }
	inline iterator end() { return table.end(); }
	iterator find(const DPState& state);
	DPSolution& operator[](const DPState& state);
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
	// Initialize and configure
	DPGeometry(const PosedCamera& camera, double zfloor, double zceil);

	// Accessors
	int nx() const { return grid_size[0]; }
	int ny() const { return grid_size[1]; }

	// Configure the various homographies and useful vanishing point info
	void Configure(const PosedCamera& camera, const Mat3& floorToCeil);
	// Configurethe various homographies and useful vanishing point info
	void Configure(const PosedCamera& camera, double zfloor, double zceil);

	// Convert between image and grid coordinates
	Vec3 GridToImage(const Vec2& x) const;
	Vec2 ImageToGrid(const Vec3& x) const;

	// Transform data from image to grid coordinates. Each element of
	// OUT is a sum over pixels from IN that project there according to
	// ImageToGrid(). No normalization is applied.
	void TransformDataToGrid(const MatF& in, MatF& out) const;

	// Transfer a point between the floor and ceiling (is always self-inverting)
	Vec2 Transfer(const Vec2& grid_pt) const;
	Vec3 Transfer(const Vec3& grid_pt) const;

	// Get the top and bottom of the wall corresponding to a given grid point
	void GetWallExtent(const Vec2& grid_pt, int axis, int& y0, int& y1) const;
	// Transform a path to an orientation map in the grid domain
	void PathToOrients(const VecI& path, const VecI& path_axes, MatI& grid_orients) const;
};

////////////////////////////////////////////////////////////////////////////////


// Represents a cost function that ManhattanDP optimizes, in terms of
// the cost of assigning label A to pixel [Y,X] (stored in
// pixel_scores[A][Y][X]). An object of this form is converted to the
// more general representation of DPPayoffs
class DPObjective {
public:
	double wall_penalty;  // the cost per wall segment (for regularisation)
	double occl_penalty;  // the cost per occluding wall segment (_additional_ to wall_penalty)
	MatF pixel_scores[3];  // score associated with assigning each label to each pixel (image coords)

	DPObjective() : wall_penalty(-1), occl_penalty(-1) { }
	DPObjective(Vec2I size) {
		Resize(size[0], size[1]);
	}
	DPObjective(int nx, int ny) {
		Resize(nx, ny);
	}

	// Get size
	int nx() const { return pixel_scores[0].Cols(); }
	int ny() const { return pixel_scores[0].Rows(); }

	// Change size
	void Resize(int nx, int ny) {
		for (int i = 0; i < 3; i++) {
			pixel_scores[i].Resize(ny, nx);
		}
	}

	// Deep copy
	void CopyTo(DPObjective& rhs) {
		for (int i = 0; i < 3; i++) {
			rhs.pixel_scores[i] = pixel_scores[i];
		}
		rhs.wall_penalty = wall_penalty;
		rhs.occl_penalty = occl_penalty;
	}
private:
	// Disallow copy constructor (CopyTo explicitly)
	DPObjective(const DPObjective& rhs);
};



////////////////////////////////////////////////////////////////////////////////
// The cost function the DP optimizes, specified in terms of the cost
// of placing the top/bottom of a wall at each pixel.
class DPPayoffs {
public:
	double wall_penalty;  // the cost per wall segment (for regularisation)
	double occl_penalty;  // the cost per occluding wall segment (_additional_ to wall_penalty)
	MatF wall_scores[2];  // cost of building the top/bottom of a wall at p (grid coords)

	// Initialize empty
	DPPayoffs();
	// Initialize and allocate
	DPPayoffs(Vec2I size);
	// Resize the score matrix
	void Resize(Vec2I size);
	// Resize the score matrix and reset all elements to the specified value
	void Resize(Vec2I size, float fill);
	// Clone this object
	void CopyTo(DPPayoffs& other);
	// Add a payoff matrix to wall_scores[0] and wall_scores[1], multiplied by a constant.
	void Add(const MatF& delta, double weight=1.0);
private:
	// Disallow copy constructor (use CopyTo explicitly instead)
	DPPayoffs(const DPPayoffs& rhs);
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

	// Various pieces of geometry
	const DPGeometry* geom; // an input parameter
	//MatI opp_rows;  // cache of floor<->ceil mapping as passed through floorToCeil

	// The cache of DP evaluations
	DPCache cache;

	// The solution and derived quantities
	DPSolution solution;  // the solution node
	vector<const DPState*> full_backtrack;  // series of nodes to the solution
	vector<const DPState*> abbrev_backtrack;  // as above but omitting UP, DOWN, and some OUT nodes
	vector<LineSeg> soln_segments;  // line segments represent either the top or the bottom of walls
	vector<int> soln_seg_orients;  // orientations of the above line segments
	//vector<ManhattanWall> soln_walls; // series of quads representing the solution in image coords
	//vector<ManhattanWall> soln_grid_walls; // series of quads representing the solution in grid coords
	//vector<pair<Polygon<4>, int> soln_walls;  // (Quad,Orientation) pairs comprising the solution in image coords
	MatI soln_orients;  // orientation map as predicted by the optimal model
	int soln_num_walls;  // number of walls in the solution (both normal and occluding)
	int soln_num_occlusions;  // number of occlusions in the solution
	SimpleRenderer renderer;  // used in ComputeSolutionDepth

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
	void ComputeBacktrack();
	// Get a mask in the grid domain representing the path taken through
	// the payoff matrix, with values of 0 or 1 indicating the base of
	// walls, and all other cells set to -1.
	void ComputeSolutionPath(MatI& m) const;
	void ComputeSolutionPathOld(MatI& grid) const;
	// Get a mask as above, but represented a mapping from columns to
	// the (unique) row at which the path crosses that colum, together
	// with the orientation for each image column.
	void ComputeSolutionPath(VecI& path_rows, VecI& path_orients) const;
	// Compute depth map for the solution given floor and ceiling heights
	// This returns a reference to an internal buffer.
	const MatD& ComputeDepthMap(double zfloor, double zceil);

	// Get the exact pixel-wise orientations for the current solution,
	// in grid coordinates. Uses GetSolutionPath etc.
	void ComputeGridOrients(MatI& grid_orients);
	// Get the exact pixel-wise orientations, in image coordinates, for
	// the current solution. This approach is slow because we project
	// each pixel individually into grid coordates to determine its
	// orientation.
	void ComputeExactOrients(MatI& orients);

	// The caching wrapper for the DP
	const DPSolution& Solve(const DPState& state);
	// The DP implementation
	DPSolution Solve_Impl(const DPState& state);

	// Get the marginal cost of building a wall across an additional column
	//double MarginalWallScore(int row, int col, int axis);
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
class MonocularPayoffGen {
public:
	DPGeometry geom;
	IntegralColImage<float> integ_scores[3];
	double wall_penalty, occl_penalty;
	DPPayoffs payoffs;  // payoffs are stored here when Compute() is called

	// Initialize empty
	MonocularPayoffGen() { }
	// Initialize and compute
	MonocularPayoffGen(const DPObjective& obj, const DPGeometry& geom);
	// Transform a DPObjective to a DPPayoff, storing the result in
	// this->payoffs. Equivalent to calling Configure(obj, geom) then
	// GetPayoffs(this->payoffs).
	void Compute(const DPObjective& obj,
							 const DPGeometry& geom);
	// Compute the integral images in preparation for calls to GetPayoff
	void Configure(const DPObjective& obj,
								 const DPGeometry& geom);
	// Return true iff not yet configured
	bool Empty() const;
	// Get payoff for building a wall at a point in grid coordinates.
	// grid_pt can be outside the grid bounds, clamping will be applied appropriately
	double GetWallScore(const Vec2& grid_pt, int orient) const;
	// Resize the matrix to the size of the grid and fill it with all payoffs
	void GetPayoffs(DPPayoffs& payoffs) const;
};




////////////////////////////////////////////////////////////////////////////////
// Wraps ManhattanDP, providing visualization routines that utilize a provided image
class ManhattanDPReconstructor {
public:
	const PosedImage* input;
	ManhattanDP dp;
	DPGeometry geometry;
	MonocularPayoffGen payoff_gen;  // transforms DPObjective objects to DPPayoffs objects

	// Compute the reconstruction for the given objective
	// function. Internally the DPObjective is converted to a DPPayoffs
	// using MonocularPayoffGen before passing it to ManhattanDP
	void Compute(const PosedImage& image,
							 const Mat3& floorToCeil,
							 const DPObjective& scores);
	// Compute the reconstruction for the given payoff function
	void Compute(const PosedImage& image,
							 const DPGeometry& geometry,
	             const DPPayoffs& payoffs);

	// Report the solution as a sequence of DP nodes
	void ReportBacktrack();

	// Compute classification accuracy w.r.t. ground truth
	double GetAccuracy(const MatI& gt_orients);
	// Compute classification accuracy w.r.t. the ground truth floorplan
	double GetAccuracy(const proto::FloorPlan& gt_floorplan);
	// Report and return classification accuracy.
	double ReportAccuracy(const proto::FloorPlan& gt_floorplan);

	// Compute mean relative-depth-error
	double GetDepthError(const proto::FloorPlan& gt_floorplan);
	// Report and return mean relative-depth-error
	double ReportDepthError(const proto::FloorPlan& gt_floorplan);

	// Draw the original image
	void OutputOrigViz(const string& path);
	// Output the solution as an image blended with the solultion orientations
	void OutputSolution(const string& path);
	// Draw the solution in grid coordinates
	void OutputGridViz(const string& path);
	// Draw a visualization of the floor-to-ceiling mapping
	void OutputManhattanHomologyViz(const string& path);
};

}
