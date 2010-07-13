#include <tr1/unordered_map>

#include <LU.h>

#include "manhattan_dp.h"
#include "common_types.h"
#include "map.h"
#include "camera.h"
#include "timer.h"
#include "floor_ceil_map.h"
#include "clipping.h"
#include "canvas.h"
#include "bld_helpers.h"

#include "fill_polygon.tpp"
#include "integral_col_image.tpp"
#include "math_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"
#include "range_utils.tpp"
#include "vector_utils.tpp"
#include "histogram.tpp"

namespace indoor_context {
using namespace toon;

lazyvar<int> gvMaxComplexity("ManhattanDP.MaxCorners");
lazyvar<int> gvGridScale("ManhattanDP.OrientSampleFactor");
lazyvar<float> gvGridYOffsetRel("ManhattanDP.ExtraSearchMargin");
lazyvar<int> gvVerticalAxis("ManhattanDP.VerticalAxis");
lazyvar<float> gvLineJumpThreshold("ManhattanDP.LineJumpThreshold");
lazyvar<float> gvCornerPenalty("ManhattanDP.CornerPenalty");







DPState::DPState() : row(-1), col(-1), axis(-1), remaining(-1), dir(-1) { }
DPState::DPState(int r, int c, int a, int b, int d)
: row(r), col(c), axis(a), remaining(b), dir(d) {	}
bool DPState::operator==(const DPState& other) const {
	return
	row == other.row &&
	col == other.col &&
	axis == other.axis &&
	remaining == other.remaining &&
	dir == other.dir;
}
bool DPState::operator!=(const DPState& other) const {
	return
	row != other.row ||
	col != other.col ||
	axis != other.axis ||
	remaining != other.remaining ||
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
	s << "{r=" << x.row << ",c=" << x.col << ",axis=" << x.axis
			<< ",nr=" << x.remaining << ",dir=" << (x.dir==DPState::DIR_IN?"DIR_IN":"DIR_OUT") << "}";
	return s;
}

ostream& operator<<(ostream& s, const DPSolution& x) {
	s << "<score=" << x.score << ",src=" << x.src << ">";
	return s;
}


void DPCache::reset(const Vector<2,int>& grid_size, int max_corners) {
	// This ordering helps the OS to do locality-based caching
	// note: we _always_ resize because it implicitly resets the cache elements
	table.Resize(max_corners+1, grid_size[0], grid_size[1], 2, 4);
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
	return table(x.remaining, x.col, x.row, x.axis, x.dir);
}






ManhattanDP::ManhattanDP() {
	jump_thresh = *gvLineJumpThreshold;
	vert_axis = *gvVerticalAxis;

	max_corners = *gvMaxComplexity;
	corner_penalty = *gvCornerPenalty;

	grid_scale_factor = *gvGridScale;
	grid_offset_factor = *gvGridYOffsetRel;
}

void ManhattanDP::Compute(const MatI& orients, const PosedCamera& cam, const Mat3& fcmap) {
	pc = &cam;  // the camera
	floorToCeil = fcmap;  // the mapping between floor and ceiling
	input_orients = &orients;
	input_size = makeVector(orients.Cols(), orients.Rows());

	ComputeGridWarp();  // note this _must_ come before everything else
	ComputeOrients();
	ComputeOppositeRows();

	// Reset the profiling histograms
	horiz_len_hist.Clear();
	horiz_cutoff_hist.Clear();

	// Reset the cache
	cache_lookups = 0;
	cache_hits = 0;

	// Reset the cache
	cache.reset(grid_size, max_corners);

	// Begin the search
	DPSolution best(-INFINITY);
	DPState init(-1, orient_map.Cols()-1, -1, -1, DPState::DIR_OUT);
	max_depth = cur_depth = 0;
	VW::Timer dp_timer;
	for (init.remaining = 1; init.remaining <= max_corners; init.remaining++) {
		int penalty = corner_penalty * init.remaining;
		for (init.axis = 0; init.axis <= 1; init.axis++) {
			for (init.row = 0; init.row < orient_map.Rows(); init.row++) {
				best.ReplaceIfSuperior(Solve(init), init, -penalty);
			}
		}
	}
	solve_time = dp_timer.GetAsMilliseconds();
	DLOG << "Solve DP: " << solve_time << "ms";

	// Store the optimal solution
	solution = best;

	// Backtrack from the solution
	ComputeBacktrack();
}

void ManhattanDP::ComputeGridWarp() {
	// Build a rotation to move the vertical vanishing point to infinity.
	// R must satisfy:
	//   (1) R*up = [0,1,0]
	//   (2) R is as close to the identity as possible
	//        (so that the other vpts are minimally affected)
	Vec3 up = pc->GetRetinaVpt(2);
	Mat3 R_up;
	R_up[0] = up ^ GetAxis<3>(2);
	R_up[1] = up;
	R_up[2] = R_up[0] ^ R_up[1];

	// T_centre projects the centre of the original image projects to
	// the centre of the new image
	Mat3 T_centre = Identity;
	T_centre.slice<0,2,2,1>() = -project(R_up*makeVector(0,0,1)).as_col();

	// Compute the warping homographies to transform the image so that
	// vertical in the world is vertical in the image.
	Mat3 H_rect = T_centre * R_up;

	// Compute scaling to keep all corners within bounds
	double scale = 1.0;
	const Bounds2D<>& ret_bounds = pc->camera.ret_bounds();
	Polygon<4> ret_perimeter = ret_bounds.GetPolygon();
	for (int i = 0; i < 4; i++) {
		double canon_x = project(H_rect * ret_perimeter.verts[i])[0];
		// this works because the retina is centred at zero...
		scale = max(scale, canon_x/ret_bounds.left);
		scale = max(scale, canon_x/ret_bounds.right);
	}
	DiagonalMatrix<3> M_scale(makeVector(1.0/scale, 1.0/scale, 1.0));

	// Compute offsets and scaling applied after the warp
	double grid_scaling = grid_scale_factor;
	Vec2 grid_offset = makeVector(0, input_size[1] * grid_offset_factor/grid_scaling);
	grid_size = input_size/grid_scaling + grid_offset*2;

	// Make the grid transform as a matrix
	Mat3 grid_xform = Identity;
	grid_xform[0][0] = 1.0/grid_scaling;
	grid_xform[1][1] = 1.0/grid_scaling;
	grid_xform.slice<0,2,2,1>() = grid_offset.as_col();

	// Approximate the camera as linear
	Mat3 intr = pc->camera.Linearize();
	Mat3 intr_inv = LU<>(intr).get_inverse();

	// Compose the whole thing
	imageToGrid = grid_xform * intr * H_rect * M_scale * intr_inv;
	gridToImage = LU<>(imageToGrid).get_inverse();

	// Locate the vanishing points in grid coordinates
	for (int i = 0; i < 2; i++) {
		horiz_vpts[i] = ImageToGrid(pc->GetImageVpt(i));
	}

	// Note that H_canon breaks the orthogonality of the vanishing
	// points, so the horzon is not guaranteed to be in the middle of
	// the image even though the vertical vanishing point is
	// guaranteed to be at infinity. The horizon is, however,
	// guaranteed to be horizontal in the image.
	CHECK_TOL(horiz_vpts[0][1], horiz_vpts[1][1], 1e-6)
	<< "The horizon should be horizontal in the image. Perhaps H_canon is wrong?"
	<< "the vpts are: " << horiz_vpts[0] << " and " << horiz_vpts[1];

	// Calculate the row the horizon is in
	horizon_row = roundi(horiz_vpts[0][1]);
}

void ManhattanDP::ComputeOrients() {
	const MatI& orients = *input_orients;

	// Setup the votes
	MatI votes[3];
	for (int i = 0; i < 3; i++) {
		votes[i].Resize(grid_size[1], grid_size[0], 0);
	}

	// Accumulate votes (accounting for warping, downsampling, and grid_offset)
	MatI canon_orients(orients.Rows(), orients.Cols());
	for (int y = 0; y < orients.Rows(); y++) {
		const int* orientrow = orients[y];
		for (int x = 0; x < orients.Cols(); x++) {
			if (orientrow[x] != -1) {
				Vec2I grid_pt = ImageToGrid(makeVector(x, y, 1.0));
				if (grid_pt[0] >= 0 && grid_pt[0] < grid_size[0] &&
						grid_pt[1] >= 0 && grid_pt[1] < grid_size[1]) {
					votes[ orientrow[x] ][ grid_pt[1] ][ grid_pt[0] ]++;
				}
			}
		}
	}

	// Resolve votes into a final orientation map
	orient_map.Resize(grid_size[1], grid_size[0], -1);
	int cell_area = (input_size[0]*input_size[1]) / (grid_size[0]*grid_size[1]);
	for (int y = 0; y < grid_size[1]; y++) {
		int* outrow = orient_map[y];
		int* voterow[] = { votes[0][y], votes[1][y], votes[2][y] };
		for (int x = 0; x < grid_size[0]; x++) {
			outrow[x] = -1;
			int best_vote = cell_area/10;  // below this threshold pixels will be "unknown"
			for (int i = 0; i < 3; i++) {
				if (voterow[i][x] > best_vote) {
					best_vote = voterow[i][x];
					outrow[x] = i;
				}
			}
		}
	}

	// Compute the integral-col image
	integ_orients.Compute(orient_map);
}

void ManhattanDP::ComputeOppositeRows() {
	opp_rows.Resize(orient_map.Rows(), orient_map.Cols());
	Mat3 grid_floorToCeil = imageToGrid * floorToCeil * gridToImage;  // floorToCeil in grid coords
	Mat3 grid_ceilToFloor = LU<>(grid_floorToCeil).get_inverse();
	for (int y = 0; y < opp_rows.Rows(); y++) {
		const Mat3& m = y < horizon_row ? grid_ceilToFloor : grid_floorToCeil;
		for (int x = 0; x < opp_rows.Cols(); x++) {
			Vec2 opp_grid_pos = project(m * makeVector(x,y,1.0));
			// TODO: is Clamp appropriate here??
			opp_rows[y][x] = Clamp<int>(opp_grid_pos[1], 0, orient_map.Rows()-1);
		}
	}
}

const DPSolution& ManhattanDP::Solve(const DPState& state) {
	cache_lookups++;
	max_depth = max(max_depth, cur_depth);

	// unordered_map::insert actually does a lookup first and returns
	// an existing element if there is one, or returns an iterator
	// pointing to a newly inserted element if not.
	Cache::iterator it = cache.find(state);
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

	} else if (state.remaining == 0) {
		// base case 2
		best.score = -INFINITY;

	} else if (state.dir == DPState::DIR_IN) {
		DPState next = state;
		for (next.axis = 0; next.axis <= 1; next.axis++) {
			// Try going out from this point directly
			next.dir = DPState::DIR_OUT;
			best.ReplaceIfSuperior(Solve(next), next);

			// Try going up from here
			next.dir = DPState::DIR_UP;
			if (CanMoveVert(state, next)) {
				best.ReplaceIfSuperior(Solve(next), next);
			}

			// Try going down from here
			next.dir = DPState::DIR_DOWN;
			if (CanMoveVert(state, next)) {
				best.ReplaceIfSuperior(Solve(next), next);
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
		if (next_row != horizon_row && next_row >= 0 && next_row < orient_map.Rows()) {
			DPState next = state;
			next.row = next_row;
			best.ReplaceIfSuperior(Solve(next), next);
		}

	} else if (state.dir == DPState::DIR_OUT) {
		DPState next = state;
		next.dir = DPState::DIR_IN;
		next.remaining = state.remaining-1;

		double score_delta = 0.0;
		const Vec2& vpt = horiz_vpts[state.axis];

		double m = (state.row-vpt[1])/(state.col-vpt[0]);
		double c = vpt[1] - m*vpt[0];

		//horiz_len_hist.Add(state.col-1); // for profiling
		for (next.col = state.col-1; next.col >= 0; next.col--) {
			// Check that we don't cross the vpt
			if (abs(vpt[0] - next.col) < 1.0) break;

			// Compute the new row
			double next_y = m*next.col + c;
			next.row = roundi(next_y);

			// Check bounds.
			if (next.row < 0 || next.row >= orient_map.Rows()) continue;

			// Compute the row of the opposite face (floor <-> ceiling)
			// TODO: ComputeOppositeRows clamps the values in opp_rows, review
			// whether this is appropriate.
			int opp_row = opp_rows[next.row][next.col];

			// Compute the score for this segment
			// TODO: this is the cost function: factor it out
			int wall_orient = 1-next.axis;  // labels refer to normal direction rather than vpt index
			int r0 = min(next.row, opp_row);
			int r1 = max(next.row, opp_row);
			score_delta += integ_orients.Count(vert_axis, next.col, 0, r0)
							+ integ_orients.Count(wall_orient, next.col, r0, r1)
							+ integ_orients.Count(vert_axis, next.col, r1, orient_map.Rows()-1);

			// Recurse
			best.ReplaceIfSuperior(Solve(next), next, score_delta);

			// Compute the error associated with jumping to the nearest (integer-valued) pixel
			double jump_error = abs(next.row - next_y);
			double dist = abs(next.row-state.row)+abs(next.col-state.col);  // L1 norm for efficiency
			double rel_jump_error = jump_error / dist;

			// If the error is sufficiently small then allow the line to
			// continue with a slight aberation. This approximation
			// reduces complexity from O( W*H*(W+H) ) to O(W*H)
			if (rel_jump_error < jump_thresh) {
				horiz_cutoff_hist.Add(state.col - next.col);
				// we just continue from this point -- don't add an intersection
				next.dir = DPState::DIR_OUT;
				next.remaining = state.remaining;
				best.ReplaceIfSuperior(Solve(next), next, score_delta);
				// This recursion has already (approximately)
				// considered all further points along the line so there is
				// no need to continue. Note that we break here
				// regardless of whether this solution replaced the
				// best so far because if this solution did _not_ replace
				// the best so far then nothing along this line will
				// do so.
				break;
			}

		}
	}

	return best;
}

void ManhattanDP::ComputeBacktrack() {
	soln_walls.clear();
	soln_grid_walls.clear();
	soln_orients.Resize(input_size[1], input_size[0], 2);

	// Trace the solution
	const DPState* cur = &solution.src;
	const DPState* out = NULL;
	do {
		full_backtrack.push_back(cur);

		if (cur->dir == DPState::DIR_IN && out != NULL) {
			Vec2 tl = cur->position();
			Vec2 tr = out->position();
			Vec2 bl = makeVector(tl[0], opp_rows[ cur->row ][ cur->col ]);
			Vec2 br = makeVector(tr[0], opp_rows[ out->row ][ out->col ]);

			soln_grid_walls.push_back(ManhattanWall(unproject(tl),
					unproject(tr),
					unproject(br),
					unproject(bl),
					out->axis));
			soln_walls.push_back(ManhattanWall(GridToImage(tl),
					GridToImage(tr),
					GridToImage(br),
					GridToImage(bl),
					out->axis));

			vector<Vec3 > poly;
			copy_all_into(soln_walls.back().poly.verts, poly);
			int wall_orient = 1-out->axis;  // labels refer to normal direction rather than vpt index
			FillPolygon(poly, soln_orients, wall_orient);

			abbrev_backtrack.push_back(cur);
			out = NULL;
		} else if (cur->dir == DPState::DIR_OUT && out == NULL) {
			abbrev_backtrack.push_back(cur);
			out = cur;
		}

		cur = &cache[*cur].src;
	} while (*cur != DPState::none);
}


Vec3 ManhattanDP::GridToImage(const Vec2& x) {
	return gridToImage * unproject(x);
}

Vec2 ManhattanDP::ImageToGrid(const Vec3& x) {
	return project(imageToGrid * x);
}

bool ManhattanDP::OcclusionValid(int col, int left_axis, int right_axis, int occl_side) {
	int occl_axis = occl_side < 0 ? left_axis : right_axis;
	int occl_vpt_col = horiz_vpts[occl_axis][0];
	int occl_vpt_side = occl_vpt_col < col ? -1 : 1;

	int opp_axis = 1-occl_axis;  // irrespective of whether left_axis=right_axis!
	int opp_vpt_col = horiz_vpts[opp_axis][0];
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
	bool on_ceil = cur.row < horizon_row;
	bool going_up = next.dir == DPState::DIR_UP;
	int occl_side = (on_ceil == going_up ? -1 : 1);

	// Here we assume that we're moving right-to-left
	return OcclusionValid(cur.col, next.axis, cur.axis, occl_side);
}


void ManhattanDP::DrawWalls(ImageRGB<byte>& canvas,
                            const vector<ManhattanWall>& walls) const {
	BrightColors bc;
	BOOST_FOREACH(const ManhattanWall& wall, walls) {
		// Draw a quad with a cross through it
		PixelRGB<byte> color = bc.Next();
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < i; j++) {
				DrawLineClipped(canvas,
						LineSeg(wall.poly.verts[i], wall.poly.verts[j]),
						color);
			}
		}
	}
}


void ManhattanDP::DrawSolution(ImageRGB<byte>& canvas) const {
	DrawWalls(canvas, soln_walls);
}

void ManhattanDP::DrawGridSolution(ImageRGB<byte>& canvas) const {
	DrawOrientations(orient_map, canvas);
	DrawWalls(canvas, soln_grid_walls);
}






void ManhattanDPReconstructor::Compute(const PosedImage& pim,
                                       const proto::TruthedMap& tru_map) {
	input = &pim;

	// Detect lines and line sweeps
	TIMED("Detect lines")
		line_detector.Compute(pim);
	TIMED("Estimate orientations")
		line_sweeper.Compute(pim, line_detector.detections);

	// TODO: do this with a line detector
	double zfloor = tru_map.floorplan().zfloor();
	double zceil = tru_map.floorplan().zceil();
	Vec3 vup = pim.pc.pose.inverse() * makeVector(0,1,0);
	if (Sign(zceil-zfloor) == Sign(vup[2])) {
		swap(zfloor, zceil);
	}
	Mat3 floorToCeil = GetManhattanHomology(pim.pc, zfloor, zceil);

	// Run the DP
	TIMED("Complete DP") dp.Compute(line_sweeper.orient_map, pim.pc, floorToCeil);
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
	GetTrueOrients(gt_floorplan, input->pc, gt_orients);
	return GetAccuracy(gt_orients);
}

void ManhattanDPReconstructor::OutputOrigViz(const string& path) {
	WriteImage(path, input->rgb);
}

void ManhattanDPReconstructor::OutputOrientViz(const string& path) {
	ImageRGB<byte> orient_canvas;
	ImageCopy(input->rgb, orient_canvas);
	line_sweeper.DrawOrientViz(orient_canvas);
	line_detector.DrawSegments(orient_canvas);
	WriteImage(path, orient_canvas);
}

void ManhattanDPReconstructor::OutputLineViz(const string& path) {
	FileCanvas canvas(path, asToon(input->sz()));
	canvas.DrawImage(input->rgb);
	canvas.SetLineWidth(3.0);
	for (int i = 0; i < 3; i++) {
		BOOST_FOREACH(const LineDetection& det, line_detector.detections[i]) {
			canvas.StrokeLine(det.seg, Colors::primary(i));
		}
	}
}

void ManhattanDPReconstructor::OutputSolutionOrients(const string& path) {
	ImageRGB<byte> soln_canvas;
	ImageCopy(input->rgb, soln_canvas);
	DrawOrientations(dp.soln_orients, soln_canvas, 0.35);
	WriteImage(path, soln_canvas);
}

void ManhattanDPReconstructor::OutputGridViz(const string& path) {
	ImageRGB<byte> grid_canvas;
	dp.DrawGridSolution(grid_canvas);
	WriteImage(path, grid_canvas);
}

void ManhattanDPReconstructor::OutputOppRowViz(const string& path) {
	FileCanvas canvas(path, input->rgb);
	for (int y = 5; y < input->ny(); y += 100) {
		for (int x = 5; x < input->nx(); x += 100) {
			Vec2 v = makeVector(x,y);
			Vec2 grid = dp.ImageToGrid(unproject(v));
			int opp_row = dp.opp_rows[ roundi(grid[1]) ][ roundi(grid[0]) ];
			Vec2 opp = project(dp.GridToImage(makeVector(grid[0], opp_row)));

			bool ceil = grid[1] < dp.horizon_row;
			PixelRGB<byte> color = ceil ? Colors::blue() : Colors::green();

			canvas.DrawDot(v, 5, Colors::white());
			canvas.DrawDot(v, 4, color);
			canvas.DrawDot(opp, 2, color);
			canvas.StrokeLine(v, opp, color);
		}
	}

	// Draw the horizon line
	Vec2 horizon_l = project(dp.GridToImage(makeVector(0, dp.horizon_row)));
	Vec2 horizon_r = project(dp.GridToImage(makeVector(input->nx(), dp.horizon_row)));
	canvas.StrokeLine(horizon_l, horizon_r, Colors::white());
}
}


/*
void ManhattanDPReconstructor::GetAuxOrients(const PosedCamera& aux,
                                             double zfloor,
                                             MatI& aux_orients) {
	aux_orients.Resize(aux.im_size().y, aux.im_size().x);
	dp.TransferBuilding(manhattan_bnb.soln, aux.pose, zfloor, aux_orients);
}

void ManhattanDPReconstructor::OutputSolutionInView(const string& path,
                                                    const Frame& aux,
                                                    double zfloor) {
	// Generate the orientation map
	MatI aux_orients;
	GetAuxOrients(*aux.pc, zfloor, aux_orients);
	CHECK(aux.image.loaded()) << "Auxiliary view not loaded";

	// Draw the image
	ImageRGB<byte> aux_canvas;
	ImageCopy(aux.image.rgb, aux_canvas);
	DrawOrientations(aux_orients, aux_canvas, 0.35);
	WriteImage(path, aux_canvas);
}
*/
