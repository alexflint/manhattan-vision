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
	Table<4, DPSolution> table;
	void reset(const Vec2I& grid_size, int max_corners);
	void clear();
	iterator begin();
	iterator end();
	iterator find(const DPState& state);
	DPSolution& operator[](const DPState& state);
};

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

	// Compute the various homographies and useful vanishing point info
	void Configure(const PosedCamera* camera, const Mat3& floorToCeil);

	// Convert between image and grid coordinates
	Vec3 GridToImage(const Vec2& x) const;
	Vec2 ImageToGrid(const Vec3& x) const;

	// Transfer a point between the floor and ceiling (is always self-inverting)
	Vec2 Transfer(const Vec2& grid_pt) const;
};


// The score (negative cost) in per-pixel, per-label form.
// TODO: make this more abstract so we can implement multiple views.
class DPObjective {
public:
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
	}

	MatF pixel_scores[3];  // score associated with assigning each label to each pixel (image coords)
	double wall_penalty;  // the cost per wall segment (for regularisation)
	double occl_penalty;  // the cost per occluding wall segment (_additional_ to wall_penalty)
private:
	// Disallow copy constructor, use CopyTo explicitly instead
	DPObjective(const DPObjective& rhs);
};


// Find optimal indoor Manhattan structures by dynamic programming
class ManhattanDP {
public:
	// Scalar parameters
	// these are initialized to the respective GVar values but can be
	// modified programatically before Compute()
	double jump_thresh;
	int vert_axis;
	int max_corners;

	// Copied from the DPObjective object passed to Compute for convenience
	double wall_penalty;  // the cost per wall segment (for regularisation)
	double occl_penalty;  // the cost per occluding wall segment (_additional_ to wall_penalty)

	// Per-pixel scores. To be factored out.
	//const DPObjective* score_func;  // Pointer to input object passed to Compute()

	// These are now only here so that ComputeScores() can re-use its large buffers
	MatF grid_scores[3];  // the above, transformed into grid coordinates
	IntegralColImage<float> integ_scores[3];  // integral-column image of scores

	// The payoff matrix. Computed from the above.
	// scores accumulated over entire columns
	// has size 2 because there are only two possible wall orientations
	boost::array<MatF,2> payoffs;

	// Geometry
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

	// Performance statistics
	double solve_time;
	int cache_lookups, cache_hits, max_depth, cur_depth;
	LazyHistogram<int> horiz_len_hist;
	LazyHistogram<int> horiz_cutoff_hist;

	// Initializes input parameters from GVar values.
	ManhattanDP();

	// Compute the optimal manhattan model from a DPObjective, which
	// expresses the problem in terms of affinities between each
	// pixel/label. This function transforms score_func into a matrix of
	// "node scores" and then calls the implementation below.
	void Compute(const DPObjective& objective,
							 const DPGeometry& geometry);

	// Compute the optimal manhattan model from the per-node score
	// matrix, which expresses the problem in terms of marginal costs of
	// building walls of each orientation at each pixel.
	void Compute(const boost::array<MatF,2>& node_scores,
							 const DPGeometry& geometry,
							 double wall_penalty,
							 double occl_penalty);

	// The above two delegate here
	void ComputeInternal();

	// Populate integ_orients with rectified score data
	void ComputePayoffs(const DPObjective& objective,
											const DPGeometry& geometry);
	// Populate opp_rows according to fcmap
	void ComputeOppositeRows(const DPGeometry& geometry);
	// Backtrack through the evaluation graph from the solution
	void ComputeBacktrack();

	// The caching wrapper for the DP
	const DPSolution& Solve(const DPState& state);
	// The DP implementation
	DPSolution Solve_Impl(const DPState& state);

	// Get the marginal cost of building a wall across an additional column
	double MarginalWallScore(int row, int col, int axis);

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

// Wraps ManhattanDP, providing visualization routines that utilize a provided image
class ManhattanDPReconstructor {
public:
	const PosedImage* input_image;
	DPGeometry geometry;
	ManhattanDP dp;

	// Compute the reconstruction for the given input and cost
	void Compute(const PosedImage& image,
	             const Mat3& floorToCeil,
	             const DPObjective& scores);
	// Compute the reconstruction for the given input and cost
	void Compute(const PosedImage& image,
							 const DPGeometry& geometry,
	             const boost::array<MatF,2>& payoffs,
							 double wall_penalty,
							 double occl_penalty);

	// Report the solution as a sequence of DP nodes
	void ReportBacktrack();

	// Compute accuracy w.r.t. ground truth
	double GetAccuracy(const MatI& gt_orients);
	// Compute accuracy w.r.t. the ground truth floorplan
	double GetAccuracy(const proto::FloorPlan& gt_floorplan);

	// Draw the original image
	void OutputOrigViz(const string& path);
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

// Compute costs by sweeping lines
// TODO: move to a different file
class LineSweepDPScore {
public:
	const PosedImage* input_image;
	GuidedLineDetector line_detector;
	IsctGeomLabeller line_sweeper;
	DPObjective score_func;

	// Initialize empty
	LineSweepDPScore() { }
	// Initialize and compute
	LineSweepDPScore(const PosedImage& image) {
		Compute(image);
	}

	// Compute a DP score function by sweeping lines in the specified image
	void Compute(const PosedImage& image);

	// Draw lines and line sweeps
	void OutputOrientViz(const string& path);
	// Draw the line detections
	void OutputLineViz(const string& path);
};

}
