#pragma once

#include <boost/array.hpp>

#include "common_types.h"
#include "camera.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"
#include "map.pb.h"

#include "table.tpp"
#include "integral_col_image.tpp"
#include "histogram.tpp"

namespace indoor_context {

////////////////////////////////////////////////////////////////////////////////
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
	void ReplaceIfSuperior(const DPSolution& other,
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
	void reset(const Vec2I& grid_size, int max_corners);
	void clear();
	iterator begin();
	iterator end();
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
	DPGeometry(const PosedCamera* camera, const Mat3& floorToCeil);
	// Initialize and configure
	DPGeometry(const PosedCamera* camera, double zfloor, double zceil);

	// Configure the various homographies and useful vanishing point info
	void Configure(const PosedCamera* camera, const Mat3& floorToCeil);
	// Configurethe various homographies and useful vanishing point info
	void Configure(const PosedCamera* camera, double zfloor, double zceil);

	// Convert between image and grid coordinates
	Vec3 GridToImage(const Vec2& x) const;
	Vec2 ImageToGrid(const Vec3& x) const;

	// Transfer a point between the floor and ceiling (is always self-inverting)
	Vec2 Transfer(const Vec2& grid_pt) const;
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
	DPObjective(int nx, int ny) {
		Resize(nx, ny);
	}

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
	int max_corners;

	// The function to optimize
	const DPPayoffs* payoffs;

	// Various pieces of geometry
	const DPGeometry* geom; // an input parameter
	MatI opp_rows;  // cache of floor<->ceil mapping as passed through floorToCeil

	// The cache of DP evaluations
	DPCache cache;

	// The solution and derived quantities
	DPSolution solution;  // the solution node
	vector<const DPState*> full_backtrack;  // series of nodes to the solution
	vector<const DPState*> abbrev_backtrack;  // as above but omitting UP, DOWN, and some OUT nodes
	vector<ManhattanWall> soln_walls; // series of quads representing the solution in image coords
	vector<ManhattanWall> soln_grid_walls; // series of quads representing the solution in grid coords
	MatI soln_orients;  // orientation map as predicted by the optimal model
	int soln_num_walls;  // number of walls in the solution (both normal and occluding)
	int soln_num_occlusions;  // number of occlusions in the solution

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
	// Populate opp_rows according to fcmap
	void ComputeOppositeRows(const DPGeometry& geometry);
	// Backtrack through the evaluation graph from the solution
	void ComputeBacktrack();
	// Get a mask representing the path taken through the payoff
	// matrix. The matrix will be set to 0 or 1 to indicate the base of
	// walls with orientation 0 or 1 respectively. The remaining
	// elements will be set to -1. The result is such that
	// this->solution.score equals the sum of all the elements in the
	// payoff matrix for which m(p) is not zero.
	void ComputeSolutionPath(MatI& m) const;

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

	// Draw a series of walls as a wireframe model
	void DrawWalls(ImageRGB<byte>& canvas,
	               const vector<ManhattanWall>& walls) const;
	// Draw the best solution as a wireframe model
	void DrawSolution(ImageRGB<byte>& canvas) const;
	// Draw the best solution in grid coordinates as a wireframe model
	void DrawGridSolution(ImageRGB<byte>& canvas) const;
};




////////////////////////////////////////////////////////////////////////////////
class MonocularPayoffGen {
public:
	bool empty;
	DPGeometry geom;
	IntegralColImage<float> integ_scores[3];
	double wall_penalty, occl_penalty;
	DPPayoffs payoffs;  // payoffs are stored here when Compute() is called

	// Initialize empty
	MonocularPayoffGen() : empty(true) { }
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
	// As above but deprecated version (TODO: delete)
	/*void ComputeOld(const PosedImage& image,
									const Mat3& floorToCeil,
									const DPObjective& scores);*/
	// Compute the reconstruction for the given payoff function
	void Compute(const PosedImage& image,
							 const DPGeometry& geometry,
	             const DPPayoffs& payoffs);

	// Report the solution as a sequence of DP nodes
	void ReportBacktrack();

	// Compute accuracy w.r.t. ground truth
	double GetAccuracy(const MatI& gt_orients);
	// Compute accuracy w.r.t. the ground truth floorplan
	double GetAccuracy(const proto::FloorPlan& gt_floorplan);

	// Draw the original image
	void OutputOrigViz(const string& path);
	// DEPRECATED: just calls OutputSolution()
	void OutputSolutionOrients(const string& path);
	// Output the solution as an image blended with the solultion orientations
	void OutputSolution(const string& path);
	// Draw the solution in grid coordinates
	void OutputGridViz(const string& path);
	// Vizualize the opp_rows matrix
	void OutputOppRowViz(const string& path);
};

}
