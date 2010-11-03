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

#include "fill_polygon.tpp"
#include "integral_col_image.tpp"
//#include "numeric_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"
#include "range_utils.tpp"
#include "vector_utils.tpp"
#include "histogram.tpp"

namespace indoor_context {
using namespace toon;

lazyvar<int> gvMaxComplexity("ManhattanDP.MaxCorners");
lazyvar<Vec2> gvGridSize("ManhattanDP.GridSize");
lazyvar<float> gvLineJumpThreshold("ManhattanDP.LineJumpThreshold");

	namespace {
		lazyvar<float> gvWallPenalty("ManhattanDP.DefaultWallPenalty");
		lazyvar<float> gvOcclusionPenalty("ManhattanDP.DefaultOcclusionPenalty");
	}

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

DPSolution::DPSolution()
: score(numeric_limits<double>::quiet_NaN()), src(DPState::none) { }
DPSolution::DPSolution(double s)
: score(s), src(DPState::none) { }
DPSolution::DPSolution(double s, const DPState& state)
: score(s), src(state) { }

void DPSolution::ReplaceIfSuperior(const DPSolution& other,
                                   const DPState& state,
                                   double delta) {
	if (other.score+delta > score) {
		score = other.score+delta;
		src = state;
	}
}

ostream& operator<<(ostream& s, const DPState& x) {
	s << "{r=" << x.row
		<< ",c=" << x.col
		<< ",axis=" << x.axis
		<< ",dir=" << (x.dir==DPState::DIR_IN?"DIR_IN":"DIR_OUT") << "}";
	return s;
}

ostream& operator<<(ostream& s, const DPSolution& x) {
	s << "<score=" << x.score << ",src=" << x.src << ">";
	return s;
}


void DPCache::reset(const Vector<2,int>& grid_size, int max_corners) {
	// This ordering helps the OS to do locality-based caching
	table.Resize(grid_size[0], grid_size[1], 2, 4);
	// Resize is a no-op if the size is the same as last time, so do a clear()
	clear();
}

void DPCache::clear() {
	table.Fill(DPSolution());
}

DPCache::iterator DPCache::begin() {
	return table.begin();
}

DPCache::iterator DPCache::end() {
	return table.end();
}

inline DPCache::iterator DPCache::find(const DPState& x) {
	DPSolution& y = (*this)[x];
	return isnan(y.score) ? end() : &y;
}

inline DPSolution& DPCache::operator[](const DPState& x) {
	// This ordering helps the OS to do locality-based caching
	return table(x.col, x.row, x.axis, x.dir);
}



DPGeometry::DPGeometry() : camera(NULL) {
	grid_size = *gvGridSize;
}

DPGeometry::DPGeometry(const PosedCamera* camera, const Mat3& floorToCeil) {
	grid_size = *gvGridSize;
	Configure(camera, floorToCeil);
}

DPGeometry::DPGeometry(const PosedCamera* camera, double zfloor, double zceil) {
	grid_size = *gvGridSize;
	Configure(camera, zfloor, zceil);
}

void DPGeometry::Configure(const PosedCamera* cam, double zfloor, double zceil) {
	Configure(cam, GetManhattanHomology(*cam, zfloor, zceil));
}

void DPGeometry::Configure(const PosedCamera* cam, const Mat3& fToC) {
	camera = cam;
	floorToCeil = fToC;

	// Compute the rectification homography
	imageToGrid = GetVerticalRectifier(*camera, Bounds2D<>::FromSize(grid_size));
	gridToImage = LU<>(imageToGrid).get_inverse();

	// Compute floor to ceiling mapping in grid coordinates
	grid_floorToCeil = imageToGrid * floorToCeil * gridToImage;  // floorToCeil in grid coords
	grid_ceilToFloor = LU<>(grid_floorToCeil).get_inverse();

	// Locate the vanishing points in grid coordinates
	for (int i = 0; i < 3; i++) {
		vpt_cols[i] = ImageToGrid(cam->GetImageVpt(i))[0];
	}

	// Calculate the horizon row
	// Note that H_canon breaks the orthogonality of the vanishing
	// points, so the horzon is not guaranteed to be in the middle of
	// the image even though the vertical vanishing point is
	// guaranteed to be at infinity. The horizon is, however,
	// guaranteed to be horizontal in the image.
	double y0 = ImageToGrid(cam->GetImageVpt(0))[1];
	double y1 = ImageToGrid(cam->GetImageVpt(1))[1];
	CHECK_EQ_TOL(y0, y1, 1e-6) << "The horizon is not horizontal in the image.";
	horizon_row = roundi((y0 + y1)/2.0);

	// Check that image is not flipped. This should be guaranteed by GetVerticalRectifier
	Vec2 floor_pt = makeVector(0, horizon_row+1);
	// Note that PosedCamera::GetImageHorizon always returns a line with positive half on the floor...
	CHECK_GT(GridToImage(floor_pt) * cam->GetImageHorizon(), 0)
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
	return project(m * unproject(grid_pt));
}




ManhattanDP::ManhattanDP() : geom(NULL) {
	jump_thresh = *gvLineJumpThreshold;
	max_corners = *gvMaxComplexity;
}

void ManhattanDP::Compute(const DPObjective& objective,
													const DPGeometry& geometry) {
	geom = &geometry;
	wall_penalty = objective.wall_penalty;
	occl_penalty = objective.occl_penalty;

	ComputeOppositeRows(geometry);
	ComputePayoffs(objective, geometry);
	ComputeInternal();
}

// This is mostly a copy of Compute() above. TODO: Factor out common elements of both.
void ManhattanDP::Compute(const boost::array<MatF,2>& the_payoffs,
													const DPGeometry& geometry,
													double the_wall_penalty,
													double the_occl_penalty) {  // TODO: deal with wall_penalty better
	payoffs[0] = the_payoffs[0];  // copying, uh oh
	payoffs[1] = the_payoffs[1];
	geom = &geometry;
	wall_penalty = the_wall_penalty;
	occl_penalty = the_occl_penalty;

	ComputeOppositeRows(geometry);
	ComputeInternal();
}

void ManhattanDP::ComputeInternal() {
	CHECK_GE(wall_penalty, 0) <<
		"This indicates DPObjective::wall_penalty was not changed from "
		"the initial value set by the constructor";
	CHECK_GE(occl_penalty, 0) <<
		"This indicates DPObjective::occl_penalty was not changed from "
		"the initial value set by the constructor";

	// Reset the cache
	cache_lookups = 0;
	cache_hits = 0;
	cache.reset(geom->grid_size, max_corners);

	// Begin the search
	DPSolution best(-INFINITY);
	DPState init(-1, geom->grid_size[0]-1, -1, -1, DPState::DIR_OUT);
	max_depth = cur_depth = 0;
	TIMED("Core DP")
	for (init.axis = 0; init.axis <= 1; init.axis++) {
		for (init.row = geom->horizon_row; init.row < geom->grid_size[1]; init.row++) {
			best.ReplaceIfSuperior(Solve(init), init);
		}
	}
	//DREPORT(max_depth);

	// Backtrack from the solution
	solution = best;
	ComputeBacktrack();	
}

void ManhattanDP::ComputePayoffs(const DPObjective& objective,
																 const DPGeometry& geometry) {
	// Compute per orientation, per pixel affinities
	for (int i = 0; i < 3; i++) {
		// Allocate memory
		grid_scores[i].Resize(geometry.grid_size[1], geometry.grid_size[0], 0);
		CHECK_EQ(objective.pixel_scores[i].Rows(), geometry.camera->image_size().y);
		CHECK_EQ(objective.pixel_scores[i].Cols(), geometry.camera->image_size().x);

		// Transform the scores according to ImageToGrid(.)
		for (int y = 0; y < geometry.camera->image_size().y; y++) {
			const float* inrow = objective.pixel_scores[i][y];
			for (int x = 0; x < geometry.camera->image_size().x; x++) {
				Vec2I grid_pt = RoundVector(geometry.ImageToGrid(makeVector(x, y, 1.0)));
				if (grid_pt[0] >= 0 && grid_pt[0] < geometry.grid_size[0] &&
					grid_pt[1] >= 0 && grid_pt[1] < geometry.grid_size[1]) {
					grid_scores[i][ grid_pt[1] ][ grid_pt[0] ] += inrow[x];
				}
			}
		}

		// Compute the integral-col image
		integ_scores[i].Compute(grid_scores[i]);
	}

	// Compute the whole-column costs for each node
	for (int i = 0; i < 2; i++) {
		payoffs[i].Resize(geometry.grid_size[1], geometry.grid_size[0]);
		for (int y = 0; y < geometry.grid_size[1]; y++) {
			float* row = payoffs[i][y];
			for (int x = 0; x < geometry.grid_size[0]; x++) {
				row[x] = MarginalWallScore(y, x, i);
			}
		}
	}
}

void ManhattanDP::ComputeOppositeRows(const DPGeometry& geometry) {
	opp_rows.Resize(geometry.grid_size[1], geometry.grid_size[0]);
	for (int y = 0; y < geometry.grid_size[1]; y++) {
		const Mat3& m = y < geometry.horizon_row ?
			geometry.grid_ceilToFloor : geometry.grid_floorToCeil;
		for (int x = 0; x < geometry.grid_size[0]; x++) {
			Vec2 opp_grid_pos = project(m * makeVector(x,y,1.0));
			opp_rows[y][x] = opp_grid_pos[1];
		}
	}
}

const DPSolution& ManhattanDP::Solve(const DPState& state) {
	cache_lookups++;
	max_depth = max(max_depth, cur_depth);
	//CHECK_GE(state.row, geom->horizon_row);

	// unordered_map::insert actually does a lookup first and returns
	// an existing element if there is one, or returns an iterator
	// pointing to a newly inserted element if not.
	DPCache::iterator it = cache.find(state);
	if (it == cache.end()) {
		cur_depth++;
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
	// This function is one of the few places where
	// micro-optimizations make a real difference!
	DPSolution best(-INFINITY);
	if (state.col == 0) {
		// base case 1
		best.score = 0;

	} else if (state.dir == DPState::DIR_IN) {
		DPState next = state;
		for (next.axis = 0; next.axis <= 1; next.axis++) {
			// Try going out from this point directly
			next.dir = DPState::DIR_OUT;
			best.ReplaceIfSuperior(Solve(next), next, -wall_penalty);

			// Try going up from here
			next.dir = DPState::DIR_UP;
			if (CanMoveVert(state, next)) {
				best.ReplaceIfSuperior(Solve(next), next, -wall_penalty-occl_penalty);
			}

			// Try going down from here
			next.dir = DPState::DIR_DOWN;
			if (CanMoveVert(state, next)) {
				best.ReplaceIfSuperior(Solve(next), next, -wall_penalty-occl_penalty);
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

		// Try going up/down. Never cross the horizon
		int next_row = state.row + (state.dir == DPState::DIR_UP ? -1 : 1);
		if (next_row != geom->horizon_row && next_row >= 0 && next_row < geom->grid_size[1]) {
			DPState next = state;
			next.row = next_row;
			best.ReplaceIfSuperior(Solve(next), next);
		}

	} else if (state.dir == DPState::DIR_OUT) {
		DPState next = state;
		next.dir = DPState::DIR_IN;

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
				next.row = roundi(next_y);

				// Check bounds and that we don't cross the horizon
				if (next.row < 0 || next.row >= geom->grid_size[1] || next.row == geom->horizon_row) break;

				// Compute cost
				delta_score += payoffs[next.axis][next.row][next.col];

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

double ManhattanDP::MarginalWallScore(int row, int col, int axis) {
	// Compute the row of the opposite face (floor <-> ceiling)
	int opp_row = Clamp<int>(opp_rows[row][col], 0, geom->grid_size[1]-1);

	// Compute the score for this segment
	int wall_orient = 1-axis;  // labels refer to normal direction rather than vpt index
	int r0 = min(row, opp_row);
	int r1 = max(row, opp_row);
	return integ_scores[kVerticalAxis].Sum(col, 0, r0-1)
         + integ_scores[wall_orient].Sum(col, r0, r1-1)
         + integ_scores[kVerticalAxis].Sum(col, r1, geom->grid_size[1]-1);
}

void ManhattanDP::ComputeBacktrack() {
	soln_walls.clear();
	soln_grid_walls.clear();
	soln_orients.Resize(geom->camera->image_size().y, geom->camera->image_size().x, kVerticalAxis);

	// Trace the solution
	soln_num_walls = 0;
	soln_num_occlusions = 0;
	const DPState* cur = &solution.src;
	const DPState* out = NULL;
	do {
		const DPState& next = cache[*cur].src;

		full_backtrack.push_back(cur);
		if (cur->dir == DPState::DIR_IN && out != NULL) {
			soln_num_walls++;
			if (next.dir == DPState::DIR_UP || next.dir == DPState::DIR_DOWN) {
				soln_num_occlusions++;
			}

			Vec2 tl = makeVector(cur->col, cur->row);  // check this! was: cur->position()
			Vec2 tr = makeVector(out->col, out->row);  // check this! was: out->position
			Vec2 bl = makeVector(tl[0], opp_rows[ cur->row ][ cur->col ]);
			Vec2 br = makeVector(tr[0], opp_rows[ out->row ][ out->col ]);

			soln_grid_walls.push_back(ManhattanWall(unproject(tl),
					unproject(tr),
					unproject(br),
					unproject(bl),
					out->axis));
			soln_walls.push_back(ManhattanWall(geom->GridToImage(tl),
					geom->GridToImage(tr),
					geom->GridToImage(br),
					geom->GridToImage(bl),
					out->axis));

			vector<Vec3> poly;
			copy_all_into(soln_walls.back().poly.verts, poly);
			int wall_orient = 1-out->axis;  // labels refer to normal direction rather than vpt index
			FillPolygon(poly, soln_orients, wall_orient);

			abbrev_backtrack.push_back(cur);
			out = NULL;
		} else if (cur->dir == DPState::DIR_OUT && out == NULL) {
			abbrev_backtrack.push_back(cur);
			out = cur;
		}

		//cur = &cache[*cur].src;
		cur = &next;
	} while (*cur != DPState::none);
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

// Determine whether occlusion constrains prevent us from moving
// between two states in the DP
bool ManhattanDP::CanMoveVert(const DPState& cur, const DPState& next) {
	// is the occluding stripe to the left or right?
	bool on_ceil = cur.row < geom->horizon_row;
	bool going_up = next.dir == DPState::DIR_UP;
	int occl_side = (on_ceil == going_up ? -1 : 1);

	// Here we assume that we're moving right-to-left
	return OcclusionValid(cur.col, next.axis, cur.axis, occl_side);
}


void ManhattanDP::DrawWalls(ImageRGB<byte>& canvas,
                            const vector<ManhattanWall>& walls) const {
	BOOST_FOREACH(const ManhattanWall& wall, walls) {
		for (int i = 0; i < 4; i++) {
			DrawLineClipped(canvas, wall.poly.edge(i), Colors::red());
		}
	}
}


void ManhattanDP::DrawSolution(ImageRGB<byte>& canvas) const {
	DrawWalls(canvas, soln_walls);
}

void ManhattanDP::DrawGridSolution(ImageRGB<byte>& canvas) const {
	DrawWalls(canvas, soln_grid_walls);
}


void ManhattanDPReconstructor::Compute(const PosedImage& image,
                                       const Mat3& floorToCeil,
                                       const DPObjective& objective) {
	input_image = &image;
	geometry.Configure(&image.pc(), floorToCeil);
	TIMED("Complete DP") dp.Compute(objective, geometry);
}

void ManhattanDPReconstructor::Compute(const PosedImage& image,
																			 const DPGeometry& geom,
																			 const boost::array<MatF,2>& payoffs) {
	Compute(image, geom, payoffs, *gvWallPenalty, *gvOcclusionPenalty);
}

void ManhattanDPReconstructor::Compute(const PosedImage& image,
																			 const DPGeometry& geom,
																			 const boost::array<MatF,2>& payoffs,
																			 double wall_penalty,
																			 double occl_penalty) {
	input_image = &image;
	geometry = geom;  // we're copying here, but nothing big (yet)
	TIMED("Complete DP") dp.Compute(payoffs, geometry, wall_penalty, occl_penalty);
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
	GetTrueOrients(gt_floorplan, input_image->pc(), gt_orients);
	return GetAccuracy(gt_orients);
}

void ManhattanDPReconstructor::OutputOrigViz(const string& path) {
	WriteImage(path, input_image->rgb);
}

void ManhattanDPReconstructor::OutputSolutionOrients(const string& path) {
	// This version is now deprecated, use below
	DLOG << "ManhattanDPReconstructor::OutputSolutionOrients deprecated: use OutputSolution";
	OutputSolution(path);
}

void ManhattanDPReconstructor::OutputSolution(const string& path) {
	ImageRGB<byte> soln_canvas;
	ImageCopy(input_image->rgb, soln_canvas);
	DrawOrientations(dp.soln_orients, soln_canvas, 0.35);
	WriteImage(path, soln_canvas);
}

void ManhattanDPReconstructor::OutputGridViz(const string& path) {
	ImageRGB<byte> grid_canvas(geometry.grid_size[0], geometry.grid_size[1]);
	grid_canvas.Clear(Colors::white());
	dp.DrawGridSolution(grid_canvas);
	WriteImage(path, grid_canvas);
}

void ManhattanDPReconstructor::OutputOppRowViz(const string& path) {
	FileCanvas canvas(path, input_image->rgb);
	for (int y = 5; y < input_image->ny(); y += 100) {
		for (int x = 5; x < input_image->nx(); x += 100) {
			Vec3 v = makeVector(x,y,1.0);
			Vec2I grid = RoundVector(dp.geom->ImageToGrid(v));
			int opp_row = dp.opp_rows[ grid[1] ][ grid[0] ];
			Vec2 opp = project(dp.geom->GridToImage(makeVector(grid[0], opp_row)));

			bool ceil = grid[1] < dp.geom->horizon_row;
			PixelRGB<byte> color = ceil ? Colors::blue() : Colors::green();

			canvas.DrawDot(project(v), 5, Colors::white());
			canvas.DrawDot(project(v), 4, color);
			canvas.DrawDot(opp, 2, color);
			canvas.StrokeLine(project(v), opp, color);
		}
	}

	// Draw the horizon line
	Vec2 horizon_l = project(dp.geom->GridToImage(makeVector(0, dp.geom->horizon_row)));
	Vec2 horizon_r = project(dp.geom->GridToImage(makeVector(input_image->nx(), dp.geom->horizon_row)));
	canvas.StrokeLine(horizon_l, horizon_r, Colors::white());
}

}
