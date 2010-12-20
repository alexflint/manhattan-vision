#include <tr1/unordered_map>

#include <LU.h>

#include "manhattan_dp.h"
#include "common_types.h"
#include "map.h"
#include "camera.h"
#include "timer.h"
#include "clipping.h"
#include "canvas.h"
#include "bld_helpers.h"
#include "geom_utils.h"
#include "line_segment.h"

#include "fill_polygon.tpp"
#include "integral_col_image.tpp"
#include "numeric_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"
#include "range_utils.tpp"
#include "vector_utils.tpp"
#include "histogram.tpp"
#include "matrix_traits.tpp"

namespace indoor_context {
using namespace toon;

lazyvar<Vec2> gvGridSize("ManhattanDP.GridSize");
lazyvar<float> gvLineJumpThreshold("ManhattanDP.LineJumpThreshold");

namespace {
	lazyvar<float> gvWallPenalty("ManhattanDP.DefaultWallPenalty");
	lazyvar<float> gvOcclusionPenalty("ManhattanDP.DefaultOcclusionPenalty");
}

////////////////////////////////////////////////////////////////////////////////
DPState::DPState() : row(-1), col(-1), axis(-1), dir(-1) { }
DPState::DPState(int r, int c, int a, int b, int d)
	: row(r), col(c), axis(a), dir(d) {	}
bool DPState::operator==(const DPState& other) const {
	return
	row == other.row &&
	col == other.col &&
	axis == other.axis &&
	dir == other.dir;
}
bool DPState::operator!=(const DPState& other) const {
	return
	row != other.row ||
	col != other.col ||
	axis != other.axis ||
	dir != other.dir;
}
const DPState DPState::none;

size_t DPStateHasher::operator()(const DPState& dpstate) const {
	return tr1::_Fnv_hash<>::hash(reinterpret_cast<const char*>(&dpstate),
			sizeof(dpstate));
}

ostream& operator<<(ostream& s, const DPState& x) {
	s << "{r=" << x.row << ",c=" << x.col<< ",axis=" << x.axis << ",dir=";
	switch (x.dir) {
	case DPState::DIR_IN: s << "DIR_IN"; break;
	case DPState::DIR_OUT: s << "DIR_OUT"; break;
	case DPState::DIR_UP: s << "DIR_UP"; break;
	case DPState::DIR_DOWN: s << "DIR_DOWN"; break;
	}
	s << "}";
	return s;
}

////////////////////////////////////////////////////////////////////////////////
DPSolution::DPSolution()
: score(numeric_limits<double>::quiet_NaN()), src(DPState::none) { }
DPSolution::DPSolution(double s)
: score(s), src(DPState::none) { }
DPSolution::DPSolution(double s, const DPState& state)
: score(s), src(state) { }

bool DPSolution::ReplaceIfSuperior(const DPSolution& other,
                                   const DPState& state,
                                   double delta) {
	if (other.score+delta > score) {
		score = other.score+delta;
		src = state;
		return true;
	}
	return false;
}

ostream& operator<<(ostream& s, const DPSolution& x) {
	s << "<score=" << x.score << ", src=" << x.src << ">";
	return s;
}


////////////////////////////////////////////////////////////////////////////////
void DPCache::reset(const Vector<2,int>& grid_size) {
	// This ordering helps the OS to do locality-based caching
	// We have one more column in the table than in the grid
	table.Resize(grid_size[0]+1, grid_size[1], 2, 4);
	// Resize is a no-op if the size is the same as last time, so do a clear()
	clear();
}

void DPCache::clear() {
	table.Fill(DPSolution());
}

DPCache::iterator DPCache::find(const DPState& x) {
	// don't be tempted to cache the value of end() here as it will
	// change from one iteration to the next.
	iterator y = &(*this)[x];
	return isnan(y->score) ? end() : y;
}

DPSolution& DPCache::operator[](const DPState& x) {
	// This particular ordering helps the OS to do locality-based caching
	return table(x.col, x.row, x.axis, x.dir);
}



////////////////////////////////////////////////////////////////////////////////
DPGeometry::DPGeometry() : camera(NULL) {
	grid_size = *gvGridSize;
}

DPGeometry::DPGeometry(const PosedCamera& camera, double zfloor, double zceil) {
	grid_size = *gvGridSize;
	Configure(camera, zfloor, zceil);
}

DPGeometry::DPGeometry(const PosedCamera& camera, const Mat3& floorToCeil) {
	grid_size = *gvGridSize;
	Configure(camera, floorToCeil);
}

void DPGeometry::Configure(const PosedCamera& cam, double zfloor, double zceil) {
	Configure(cam, GetManhattanHomology(cam, zfloor, zceil));
}

void DPGeometry::Configure(const PosedCamera& cam, const Mat3& fToC) {
	camera = &cam;
	floorToCeil = fToC;

	// Compute the rectification homography
	imageToGrid = GetVerticalRectifier(*camera, Bounds2D<>::FromTightSize(grid_size));
	gridToImage = LU<>(imageToGrid).get_inverse();

	// Compute floor to ceiling mapping in grid coordinates
	grid_floorToCeil = imageToGrid * floorToCeil * gridToImage;  // floorToCeil in grid coords
	grid_ceilToFloor = LU<>(grid_floorToCeil).get_inverse();

	// Locate the vanishing points in grid coordinates
	for (int i = 0; i < 3; i++) {
		vpt_cols[i] = ImageToGrid(cam.GetImageVpt(i))[0];
	}

	// Calculate the horizon row
	// Note that H_canon breaks the orthogonality of the vanishing
	// points, so the horzon is not guaranteed to be in the middle of
	// the image even though the vertical vanishing point is
	// guaranteed to be at infinity. The horizon is, however,
	// guaranteed to be horizontal in the image.
	double y0 = ImageToGrid(cam.GetImageVpt(0))[1];
	double y1 = ImageToGrid(cam.GetImageVpt(1))[1];
	CHECK_EQ_TOL(y0, y1, 1e-6) << "The horizon is not horizontal in the image.";
	horizon_row = roundi((y0 + y1)/2.0);

	// Check that image is not flipped. This should be guaranteed by GetVerticalRectifier
	Vec2 floor_pt = makeVector(0, horizon_row+1);
	// Note that PosedCamera::GetImageHorizon always returns a line with positive half on the floor...
	CHECK_GT(GridToImage(floor_pt) * cam.GetImageHorizon(), 0)
		<< "The matrix returned by GetVerticalRectifier flips the image upside down!";
}

Vec3 DPGeometry::GridToImage(const Vec2& x) const {
	return gridToImage * unproject(x);
}

Vec2 DPGeometry::ImageToGrid(const Vec3& x) const {
	return project(imageToGrid * x);
}

Vec2 DPGeometry::Transfer(const Vec2& grid_pt) const {
	const Mat3& m = grid_pt[1] < horizon_row ? grid_ceilToFloor : grid_floorToCeil;
	return project(Transfer(unproject(grid_pt)));
}

Vec3 DPGeometry::Transfer(const Vec3& grid_pt) const {
	const Mat3& m = grid_pt[1] < horizon_row ? grid_ceilToFloor : grid_floorToCeil;
	return m * grid_pt;
}

void DPGeometry::TransformDataToGrid(const MatF& in, MatF& out) const {
	CHECK_EQ(matrix_size(in), asToon(camera->image_size()));
	out.Resize(grid_size[1], grid_size[0], 0.0);

	// Check that the four corners project within the grid bounds
	CHECK_POS(ImageToGrid(makeVector(0, 0, 1)), out);
	CHECK_POS(ImageToGrid(makeVector(0, in.Rows()-1, 1)), out);
	CHECK_POS(ImageToGrid(makeVector(in.Cols()-1, 0, 1)), out);
	CHECK_POS(ImageToGrid(makeVector(in.Cols()-1, in.Rows()-1, 1)), out);

	// Do the transform
	for (int y = 0; y < in.Rows(); y++) {
		const float* inrow = in[y];
		for (int x = 0; x < in.Cols(); x++) {
			Vec2I grid_pt = RoundVector(ImageToGrid(makeVector(x, y, 1.0)));
			out[ grid_pt[1] ][ grid_pt[0] ] += inrow[x];
		}
	}
}

void DPGeometry::GetWallExtent(const Vec2& grid_pt, int axis, int& y0, int& y1) const {
	Vec2 opp_pt = Transfer(grid_pt);
	int opp_y = Clamp<int>(opp_pt[1], 0, grid_size[1]-1);

	// Rounding and clamping must come after Transfer()
	int y = Clamp<int>(roundi(grid_pt[1]), 0, grid_size[1]-1);

	// Swap if necessary
	y0 = min(y, opp_y);
	y1 = max(y, opp_y);
}

void DPGeometry::PathToOrients(const VecI& path_ys, const VecI& path_axes, MatI& grid_orients) const {
	CHECK_EQ(path_ys.Size(), grid_size[0]);
	CHECK_EQ(path_axes.Size(), grid_size[0]);
	grid_orients.Resize(grid_size[1], grid_size[0]);
	VecI y0s(grid_size[0]);
	VecI y1s(grid_size[0]);
	for (int x = 0; x < grid_size[0]; x++) {
		CHECK_NE(path_ys[x], -1) << "This should have been caught in ComputeSolutionPath";
		CHECK_NE(path_axes[x], -1) << "This should have been caught in ComputeSolutionPath";
		GetWallExtent(makeVector(x, path_ys[x]), path_axes[x], y0s[x], y1s[x]);
	}
	for (int y = 0; y < grid_size[1]; y++) {
		int* row = grid_orients[y];
		for (int x = 0; x < grid_size[0]; x++) {
			int orient = 1-path_axes[x];  // axes and orients are inverses of one another
			row[x] = (y >= y0s[x] && y <= y1s[x]) ? orient : kVerticalAxis;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
DPPayoffs::DPPayoffs()
	: wall_penalty(*gvWallPenalty), occl_penalty(*gvOcclusionPenalty) {
}

DPPayoffs::DPPayoffs(Vec2I size)
	: wall_penalty(*gvWallPenalty), occl_penalty(*gvOcclusionPenalty) {
	Resize(size);
}

void DPPayoffs::Resize(Vec2I size) {
	wall_scores[0].Resize(size[1], size[0]);
	wall_scores[1].Resize(size[1], size[0]);
}

void DPPayoffs::Resize(Vec2I size, float fill) {
	wall_scores[0].Resize(size[1], size[0], fill);
	wall_scores[1].Resize(size[1], size[0], fill);
}

void DPPayoffs::CopyTo(DPPayoffs& other) {
	other.wall_scores[0] = wall_scores[0];
	other.wall_scores[1] = wall_scores[1];
	other.wall_penalty = wall_penalty;
	other.occl_penalty = occl_penalty;
}

void DPPayoffs::Add(double weight, const MatF& delta) {
	CHECK_SAME_SIZE(delta, wall_scores[0]);
	for (int i = 0; i < 2; i++) {
		for (int y = 0; y < delta.Rows(); y++) {
			const float* in = delta[y];
			float* out = wall_scores[i][y];
			for (int x = 0; x < delta.Cols(); x++) {
				out[x] += weight*in[x];
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
ManhattanDP::ManhattanDP() : geom(NULL) {
	jump_thresh = *gvLineJumpThreshold;
}

void ManhattanDP::Compute(const DPPayoffs& po,
													const DPGeometry& geometry) {
	geom = &geometry;
	payoffs = &po;
	CHECK_GE(payoffs->wall_penalty, 0);
	CHECK_GE(payoffs->occl_penalty, 0);

	// Reset the cache
	cache_lookups = 0;
	cache_hits = 0;
	cache.reset(geom->grid_size);

	// Begin the search
	DPSolution best(-INFINITY);
	DPState init(-1, geom->grid_size[0]/*yes x-coord is _past_ the image bounds*/, -1, -1, DPState::DIR_OUT);
	max_depth = cur_depth = 0;
	bool feasible = false;
	TIMED("Core DP")
	for (init.axis = 0; init.axis <= 1; init.axis++) {
		for (init.row = geom->horizon_row; init.row < geom->grid_size[1]; init.row++) {
			// Need to account for the penalty for the first wall here since
			// Solve_Impl() adds penalties on DIR_IN nodes.
			if (best.ReplaceIfSuperior(Solve(init), init, -payoffs->wall_penalty)) {
				feasible = true;
			}
		}
	}

	CHECK(feasible) << "No feasible solution found";

	// Backtrack from the solution
	solution = best;
	ComputeBacktrack();	
}

const DPSolution& ManhattanDP::Solve(const DPState& state) {
	cache_lookups++;
	//CHECK_GE(state.row, geom->horizon_row);
	//	DREPORT(state);

	// unordered_map::insert actually does a lookup first and returns
	// an existing element if there is one, or returns an iterator
	// pointing to a newly inserted element if not.
	DPCache::iterator it = cache.find(state);
	if (it == cache.end()) {
		cur_depth++;
		max_depth = max(max_depth, cur_depth);
		const DPSolution& soln = (cache[state] = Solve_Impl(state));  // single equals sign intended!
		cur_depth--;
		return soln;
	} else {
		// The key was already in the map, return the precomputed value
		cache_hits++;
		return *it;
	}
}

DPSolution ManhattanDP::Solve_Impl(const DPState& state) {
	// This function is one of the few places where micro-optimizations
	// make a real difference.

	DPSolution best(-INFINITY);
	if (state.col == 0) {
		// base case
		best.score = 0;

	} else if (state.dir == DPState::DIR_IN) {
		DPState next = state;
		double occl_wall_penalty = payoffs->wall_penalty+payoffs->occl_penalty;
		for (next.axis = 0; next.axis <= 1; next.axis++) {
			// Try going out from this point directly
			next.dir = DPState::DIR_OUT;
			best.ReplaceIfSuperior(Solve(next), next, -payoffs->wall_penalty);

			// Try going up from here
			next.dir = DPState::DIR_UP;
			if (CanMoveVert(state, next)) {
				best.ReplaceIfSuperior(Solve(next), next, -occl_wall_penalty);
			}

			// Try going down from here
			next.dir = DPState::DIR_DOWN;
			if (CanMoveVert(state, next)) {
				best.ReplaceIfSuperior(Solve(next), next, -occl_wall_penalty);
			}
		}

	} else if (state.dir == DPState::DIR_UP || state.dir == DPState::DIR_DOWN) {
		// Convention is that state.axis now indicates the axis we
		// _must_ go out on, and the validity check has already been
		// done (see DIR_IN case)

		// Try going out
		DPState next_out = state;
		next_out.dir = DPState::DIR_OUT;
		best.ReplaceIfSuperior(Solve(next_out), next_out);

		// Try going up/down. Never cross the horizon or the image bounds.
		int next_row = state.row + (state.dir == DPState::DIR_UP ? -1 : 1);
		if (next_row != geom->horizon_row && next_row >= 0 && next_row < geom->grid_size[1]) {
			DPState next = state;
			next.row = next_row;
			best.ReplaceIfSuperior(Solve(next), next);
		}

	} else if (state.dir == DPState::DIR_OUT) {
		DPState next = state;
		next.dir = DPState::DIR_IN;

		// TODO: we could just restrict the minimum length of a wall to N
		// pixels. If the line-jump threshold is satisfied for N then we
		// avoid a loop altogether.

		double delta_score = 0.0;
		int vpt_col = geom->vpt_cols[state.axis];
		if (state.col != vpt_col) {  // don't try to reconstruct perfectly oblique surfaces

			// *** To implement nonlinear spacing of grid rows, need to
			// replace state.row here with the corresponding y coordinate
			double m = (state.row-geom->horizon_row)/static_cast<double>(state.col-vpt_col);
			double c = geom->horizon_row - m*vpt_col;

			for (next.col = state.col-1; next.col >= 0; next.col--) {
				// Check that we don't cross the vpt
				if (next.col == vpt_col) break;

				// Compute the new row
				double next_y = m*next.col + c;

				// *** To implement nonlinear spacing of grid rows, need to
				// replace roundi() here with something that finds the closest
				// row to next_y
				// TODO: linearly interpolate between payoffs rather than rounding
				next.row = roundi(next_y);

				// Check bounds and that we don't cross the horizon
				if (next.row < 0 || next.row >= geom->grid_size[1] || next.row == geom->horizon_row) break;

				// Compute cost
				delta_score += payoffs->wall_scores[next.axis][next.row][next.col];

				// Recurse
				best.ReplaceIfSuperior(Solve(next), next, delta_score);

				// Compute the error associated with jumping to the nearest (integer-valued) pixel
				double jump_error = abs(next.row - next_y); // *** for nonlinear spacing, replace next.row here and below
				double dist = abs(next.row-state.row)+abs(next.col-state.col);  // L1 norm for efficiency
				double rel_jump_error = jump_error / dist;

				// If the error is sufficiently small then allow the line to
				// continue with a slight "kink". This approximation
				// reduces overall complexity from O( W*H*(W+H) ) to O(W*H)
				if (rel_jump_error < jump_thresh) {
					// we just continue from this point -- don't add an intersection
					next.dir = DPState::DIR_OUT;
					best.ReplaceIfSuperior(Solve(next), next, delta_score);
					// The above recursion has already (approximately)
					// considered all further points along the line so there is
					// no need to continue. Note that we break here regardless
					// of whether this solution replaced the best so far because
					// if this solution did not replace the best so far then
					// nothing along this line will do so.
					break;
				}
			}
		}
	}

	return best;
}

void ManhattanDP::ComputeBacktrack() {
	full_backtrack.clear();
	abbrev_backtrack.clear();
	soln_segments.clear();
	soln_seg_orients.clear();
	soln_orients.Resize(geom->camera->image_size().y,
											geom->camera->image_size().x,
											kVerticalAxis);

	// Trace the solution
	soln_num_walls = 0;  // there is always one wall
	soln_num_occlusions = 0;
	const DPState* cur = &solution.src;
	const DPState* out = NULL;
	do {
		const DPState& next = cache[*cur].src;
		CHECK(cache.find(*cur) != cache.end()) << "a state in the backtrack has no cached solution";

		full_backtrack.push_back(cur);
		if (cur->dir == DPState::DIR_IN && out != NULL) {
			abbrev_backtrack.push_back(cur);

			soln_num_walls++;
			if (next.dir == DPState::DIR_UP || next.dir == DPState::DIR_DOWN) {
				soln_num_occlusions++;
			}

			int orient = 1-cur->axis;
			LineSeg seg(makeVector(cur->col, cur->row, 1.0),
									makeVector(out->col, out->row, 1.0));
			soln_segments.push_back(seg);
			soln_seg_orients.push_back(orient);

			Vec3 verts[] = { geom->GridToImage(project(seg.start)),
											 geom->GridToImage(project(seg.end)),
											 geom->GridToImage(geom->Transfer(project(seg.end))),
											 geom->GridToImage(geom->Transfer(project(seg.start))) };
			FillPolygon(array_range(verts, 4), soln_orients, orient);

			out = NULL;
		} else if (cur->dir == DPState::DIR_OUT && out == NULL) {
			abbrev_backtrack.push_back(cur);
			out = cur;
		}

		cur = &next;
	} while (*cur != DPState::none);
}

void ManhattanDP::ComputeSolutionPathOld(MatI& grid) const {
	grid.Resize(geom->grid_size[1], geom->grid_size[0], -1);
	for (int i = 0; i < full_backtrack.size()-1; i++) {
		const DPState& state = *full_backtrack[i];
		const DPState& next = *full_backtrack[i+1];
		CHECK(state.col <= geom->grid_size[0]);  // state.col==geom->grid_size[0] is permitted
		if (state.dir == DPState::DIR_OUT) {
			int vpt_col = geom->vpt_cols[state.axis];
			double m = (state.row-geom->horizon_row)/static_cast<double>(state.col-vpt_col);
			double c = geom->horizon_row - m*vpt_col;
			for (int x = state.col-1; x >= next.col; x--) {
				int y = roundi(m*x + c);
				grid[y][x] = state.axis;
			}
		}
	}
}

void ManhattanDP::ComputeSolutionPath(MatI& grid) const {
	VecI path_ys, path_axes;
	ComputeSolutionPath(path_ys, path_axes);
	grid.Resize(geom->grid_size[1], geom->grid_size[0], -1);
	for (int x = 0; x < grid.Cols(); x++) {
		CHECK_NE(path_ys[x], -1);
		CHECK_NE(path_axes[x], -1);
		grid[ path_ys[x] ][x] = path_axes[x];
	}
}

void ManhattanDP::ComputeSolutionPath(VecI& path_ys, VecI& path_axes) const {
	path_ys.Resize(geom->grid_size[0], -1);
	path_axes.Resize(geom->grid_size[0], -1);
	for (int i = 0; i < full_backtrack.size()-1; i++) {
		const DPState& state = *full_backtrack[i];
		const DPState& next = *full_backtrack[i+1];
		CHECK(state.col <= geom->grid_size[0]);  // state.col==geom->grid_size[0] is permitted
		if (state.dir == DPState::DIR_OUT) {
			int vpt_col = geom->vpt_cols[state.axis];
			double m = (state.row-geom->horizon_row)/static_cast<double>(state.col-vpt_col);
			double c = geom->horizon_row - m*vpt_col;
			for (int x = state.col-1; x >= next.col; x--) {
				path_ys[x] = roundi(m*x + c);
				path_axes[x] = state.axis;
			}
		}
	}

	for (int x = 0; x < geom->grid_size[0]; x++) {
		CHECK_NE(path_ys[x], -1) << "Solution misses column " << x;
		CHECK_NE(path_axes[x], -1) << "Invalid orientation at column " << x;
	}
}

void ManhattanDP::ComputeGridOrients(MatI& grid_orients) {
	VecI path_ys, path_axes;
	ComputeSolutionPath(path_ys, path_axes);
	geom->PathToOrients(path_ys, path_axes, grid_orients);
}

void ManhattanDP::ComputeExactOrients(MatI& orients) {
	MatI grid_orients;
	ComputeGridOrients(grid_orients);

	orients.Resize(geom->camera->image_size().y,
								 geom->camera->image_size().x);
	for (int y = 0; y < orients.Rows(); y++) {
		int* row = orients[y];
		for (int x = 0; x < orients.Cols(); x++) {
			Vec2I grid_pt = RoundVector(geom->ImageToGrid(makeVector(x, y, 1.0)));
			// each pixel *must* project inside the image bounds
			CHECK_INTERVAL(grid_pt[0], 0, geom->grid_size[0]-1);
			CHECK_INTERVAL(grid_pt[1], 0, geom->grid_size[1]-1);
			row[x] = grid_orients[ grid_pt[1] ][ grid_pt[0] ];
		}
	}
}

const MatD& ManhattanDP::ComputeDepthMap(double zfloor, double zceil) {
	CHECK(!soln_segments.empty()) << "ComputeDepthMap() was called before Compute()";

	// Setup some geometry
	Matrix<3,4> cam = geom->camera->Linearize();
	Matrix<3,4> grid_cam = geom->imageToGrid * cam;

	// The entire solution should be on one side of the horizon
	double y0 = project(soln_segments[0].start)[1];
	double plane_z = y0 < geom->horizon_row ? zceil : zfloor;
	double opp_z =   y0 < geom->horizon_row ? zfloor : zceil;
	Vec4 plane = makeVector(0, 0, 1, -plane_z);

	// Push each polygon through the renderer
	renderer.Configure(cam, asToon(geom->camera->image_size()));
	renderer.RenderInfinitePlane(zfloor, kVerticalAxis);
	renderer.RenderInfinitePlane(zceil, kVerticalAxis);
	BOOST_FOREACH(const LineSeg& seg, soln_segments) {
		Vec3 tl = IntersectRay(seg.start, grid_cam, plane);
		Vec3 tr = IntersectRay(seg.end, grid_cam, plane);
		Vec3 bl = makeVector(tl[0], tl[1], opp_z);
		Vec3 br = makeVector(tr[0], tr[1], opp_z);
		renderer.Render(tl, br, tr, 0);  // we're only going to use the depth buffer
		renderer.Render(tl, br, bl, 0);
	}
	return renderer.depthbuffer();
}

bool ManhattanDP::OcclusionValid(int col, int left_axis, int right_axis, int occl_side) {
	int occl_axis = occl_side < 0 ? left_axis : right_axis;
	int occl_vpt_col = geom->vpt_cols[occl_axis];
	int occl_vpt_side = occl_vpt_col < col ? -1 : 1;
	int opp_vpt_col = geom->vpt_cols[1-occl_axis];  // irrespective of whether left_axis=right_axis!
	int opp_vpt_side = opp_vpt_col < col ? -1 : 1;

	// is the occluding vpt on the same side as the occluding stripe?
	bool occl_vpt_behind = occl_side == occl_vpt_side;

	// is the opposite vpt between the div and the occluding vpt?
	bool opp_vpt_between =
			opp_vpt_side == occl_vpt_side &&
			abs(col-opp_vpt_col) < abs(col-occl_vpt_col);

	// the occlusion is valid iff occl_vpt_behind == opp_vpt_between
	return occl_vpt_behind == opp_vpt_between;
}

bool ManhattanDP::CanMoveVert(const DPState& cur, const DPState& next) {
	// is the occluding stripe to the left or right?
	bool on_ceil = cur.row < geom->horizon_row;
	bool going_up = next.dir == DPState::DIR_UP;
	int occl_side = (on_ceil == going_up ? -1 : 1);

	// Here we assume that we're moving right-to-left
	return OcclusionValid(cur.col, next.axis, cur.axis, occl_side);
}

void ManhattanDP::DrawWireframeGridSolution(ImageRGB<byte>& canvas) const {
	CHECK(!soln_segments.empty());
	BOOST_FOREACH(const LineSeg& seg, soln_segments) {
		LineSeg opp(geom->Transfer(seg.start), geom->Transfer(seg.end));
		DrawLineClipped(canvas, seg, Colors::red());
		DrawLineClipped(canvas, opp, Colors::red());
		DrawLineClipped(canvas, project(seg.start), project(opp.start), Colors::red());
		DrawLineClipped(canvas, project(seg.end), project(opp.end), Colors::red());
	}
}

void ManhattanDP::DrawWireframeSolution(ImageRGB<byte>& canvas) const {
	CHECK(!soln_segments.empty());
	BOOST_FOREACH(const LineSeg& grid_seg, soln_segments) {
		LineSeg seg(geom->GridToImage(project(grid_seg.start)),
								geom->GridToImage(project(grid_seg.end)));
		LineSeg opp(geom->Transfer(seg.start), geom->Transfer(seg.end));
		DrawLineClipped(canvas, seg, Colors::red());
		DrawLineClipped(canvas, opp, Colors::red());
		DrawLineClipped(canvas, project(seg.start), project(opp.start), Colors::red());
		DrawLineClipped(canvas, project(seg.end), project(opp.end), Colors::red());
	}
}




MonocularPayoffGen::MonocularPayoffGen(const DPObjective& obj,
																			 const DPGeometry& geom) {
	Compute(obj, geom);
}

void MonocularPayoffGen::Compute(const DPObjective& obj,
																 const DPGeometry& geom) {
	Configure(obj, geom);
	GetPayoffs(payoffs);
}

void MonocularPayoffGen::Configure(const DPObjective& obj,
																	 const DPGeometry& geometry) {
	geom = geometry;
	wall_penalty = obj.wall_penalty;
	occl_penalty = obj.occl_penalty;

	Vec2I obj_size = matrix_size(obj.pixel_scores[0]);
	bool image_coords = (obj_size == asToon(geom.camera->image_size()));
	if (!image_coords) {
		CHECK_EQ(obj_size, geometry.grid_size)
			<< "Objective must be same size as either image or grid";
	}
	
	// Compute per orientation, per pixel affinities
	MatF grid_buffer;  // don't give this a size yet
	for (int i = 0; i < 3; i++) {
		if (image_coords) {
			// Transform the scores according to ImageToGrid()
			geometry.TransformDataToGrid(obj.pixel_scores[i], grid_buffer);
			integ_scores[i].Compute(grid_buffer);
		} else {
			integ_scores[i].Compute(obj.pixel_scores[i]);
		}
	}
}

bool MonocularPayoffGen::Empty() const {
	return integ_scores[0].ny() == 0;
}

double MonocularPayoffGen::GetWallScore(const Vec2& grid_pt, int axis) const {
	CHECK_GT(integ_scores[0].m_int.Rows(), 0) << "Configure() must be called before GetPayoff()";
	CHECK_INTERVAL(grid_pt[0], 0, geom.grid_size[0]-1);

	int x = roundi(grid_pt[0]);
	int y0, y1;
	geom.GetWallExtent(grid_pt, axis, y0, y1);
	int wall_orient = 1-axis;  // orients refer to normal direction rather than vpt index

	return integ_scores[kVerticalAxis].Sum(x, 0, y0-1)
		+ integ_scores[wall_orient].Sum(x, y0, y1)
		+ integ_scores[kVerticalAxis].Sum(x, y1+1, geom.grid_size[1]-1);
}

void MonocularPayoffGen::GetPayoffs(DPPayoffs& payoffs) const {
	for (int i = 0; i < 3; i++) {
		CHECK_EQ(matrix_size(integ_scores[i]), geom.grid_size+makeVector(0,1));
	}

	payoffs.Resize(geom.grid_size);
	for (int i = 0; i < 2; i++) {
		for (int y = 0; y < geom.grid_size[1]; y++) {
			float* outrow = payoffs.wall_scores[i][y];
			for (int x = 0; x < geom.grid_size[0]; x++) {
				outrow[x] = GetWallScore(makeVector(x,y), i);
			}
		}
	}
	payoffs.wall_penalty = wall_penalty;
	payoffs.occl_penalty = occl_penalty;
}






////////////////////////////////////////////////////////////////////////////////
void ManhattanDPReconstructor::Compute(const PosedImage& image,
																			 const Mat3& floorToCeil,
																			 const DPObjective& objective) {
	input = &image;
	geometry.Configure(image.pc(), floorToCeil);
	payoff_gen.Compute(objective, geometry);
	Compute(image, geometry, payoff_gen.payoffs);
}

void ManhattanDPReconstructor::Compute(const PosedImage& image,
																			 const DPGeometry& geom,
																			 const DPPayoffs& payoffs) {
	input = &image;
	geometry = geom;  // we're copying here, but nothing big (yet)
	TIMED("Complete DP") dp.Compute(payoffs, geometry);
}

void ManhattanDPReconstructor::ReportBacktrack() {
	BOOST_FOREACH(const DPState* cur, dp.full_backtrack) {
		switch (cur->dir) {
		case DPState::DIR_UP: DLOG_N << "UP"; break;
		case DPState::DIR_DOWN: DLOG_N << "DOWN"; break;
		case DPState::DIR_IN: DLOG_N << "IN"; break;
		case DPState::DIR_OUT: DLOG_N << "OUT"; break;
		}
		DLOG << str(format(" %d,%d (%d)") % cur->row % cur->col % dp.Solve(*cur).score);
	}
}

double ManhattanDPReconstructor::GetAccuracy(const MatI& gt_orients) {
	return ComputeAgreementPct(dp.soln_orients, gt_orients);
}

double ManhattanDPReconstructor::GetAccuracy(const proto::FloorPlan& gt_floorplan) {
	MatI gt_orients;
	GetTrueOrients(gt_floorplan, input->pc(), gt_orients);
	return GetAccuracy(gt_orients);
}

double ManhattanDPReconstructor::ReportAccuracy(const proto::FloorPlan& gt_floorplan) {
	double acc = GetAccuracy(gt_floorplan);
	DLOG << format("Accuracy: %25.0f%%") % (acc*100);
	return acc;
}

double ManhattanDPReconstructor::GetDepthError(const proto::FloorPlan& gt_floorplan) {
	const MatD& soln_depth = dp.ComputeDepthMap(gt_floorplan.zfloor(), gt_floorplan.zceil());
	MatI gt_orients;
	MatD gt_depth;
	GetTrueOrients(gt_floorplan, input->pc(), gt_orients, gt_depth);
	CHECK_EQ(gt_depth.Rows(), soln_depth.Rows());
	CHECK_EQ(gt_depth.Cols(), soln_depth.Cols());
	double sum_err = 0;
	for (int y = 0; y < gt_depth.Rows(); y++) {
		const double* soln_row = soln_depth[y];
		const double* gt_row = gt_depth[y];
		for (int x = 0; x < gt_depth.Cols(); x++) {
			double rel_err = abs(soln_row[x]-gt_row[x]) / gt_row[x];
			CHECK_PRED1(isfinite, rel_err) << "true=" << gt_row[x] << ", est=" << soln_row[x];
			sum_err += rel_err;
		}
	}
	return sum_err / (gt_depth.Rows() * gt_depth.Cols());
}

double ManhattanDPReconstructor::ReportDepthError(const proto::FloorPlan& gt_floorplan) {
	double acc = GetDepthError(gt_floorplan);
	DLOG << format("Mean depth error: %25.3f%%") % (acc*100);
	return acc;
}

void ManhattanDPReconstructor::OutputOrigViz(const string& path) {
	WriteImage(path, input->rgb);
}

void ManhattanDPReconstructor::OutputSolution(const string& path) {
	ImageRGB<byte> soln_canvas;
	ImageCopy(input->rgb, soln_canvas);
	DrawOrientations(dp.soln_orients, soln_canvas, 0.35);
	WriteImage(path, soln_canvas);
}

void ManhattanDPReconstructor::OutputGridViz(const string& path) {
	ImageRGB<byte> grid_canvas(geometry.grid_size[0], geometry.grid_size[1]);
	grid_canvas.Clear(Colors::white());
	dp.DrawWireframeGridSolution(grid_canvas);
	WriteImage(path, grid_canvas);
}

}
