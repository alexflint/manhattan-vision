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
lazyvar<Vec2> gvGridSize("ManhattanDP.GridSize");
lazyvar<float> gvLineJumpThreshold("ManhattanDP.LineJumpThreshold");
lazyvar<float> gvCornerPenalty("ManhattanDP.DefaultCornerPenalty");

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
	max_corners = *gvMaxComplexity;
	grid_size = *gvGridSize;
}

void ManhattanDP::Compute(const ScoreFunction& score_f,
                          const PosedCamera& cam,
                          const Mat3& fcmap) {
	pc = &cam;  // the camera
	floorToCeil = fcmap;  // the mapping between floor and ceiling
	score_func = &score_f;

	ComputeGridTransform();  // note this _must_ come before everything else
	ComputeScores();
	//ComputeGridWarpOld()
	ComputeOppositeRows();

	// Reset the cache
	cache_lookups = 0;
	cache_hits = 0;
	cache.reset(grid_size, max_corners);

	// Reset the profiling histograms
	horiz_len_hist.Clear();
	horiz_cutoff_hist.Clear();

	// Begin the search
	DPSolution best(-INFINITY);
	DPState init(-1, grid_size[0]-1, -1, -1, DPState::DIR_OUT);
	max_depth = cur_depth = 0;
	VW::Timer dp_timer;
	for (init.remaining = 1; init.remaining <= max_corners; init.remaining++) {
		int penalty = score_func->wall_penalty * init.remaining;
		for (init.axis = 0; init.axis <= 1; init.axis++) {
			for (init.row = 0; init.row < grid_size[1]; init.row++) {
				best.ReplaceIfSuperior(Solve(init), init, -penalty);
			}
		}
	}
	solve_time = dp_timer.GetAsMilliseconds();
	DLOG << "Solve DP: " << solve_time << "ms";

	// Backtrack from the solution
	solution = best;
	ComputeBacktrack();
}

void ManhattanDP::ComputeGridTransform() {
	// TODO: this produces slightly different results to
	// ComputeGridWarpOld, find out why!

	// Compute the rectification matrix
	// should we use FromTightSize here???
	imageToGrid = GetVerticalRectifier(*pc, Bounds2D<>::FromSize(grid_size));
	gridToImage = LU<>(imageToGrid).get_inverse();

	// Locate the vanishing points in grid coordinates
	for (int i = 0; i < 3; i++) {
		vpt_cols[i] = ImageToGrid(pc->GetImageVpt(i))[0];
	}

	// Calculate the horizon row
	// Note that H_canon breaks the orthogonality of the vanishing
	// points, so the horzon is not guaranteed to be in the middle of
	// the image even though the vertical vanishing point is
	// guaranteed to be at infinity. The horizon is, however,
	// guaranteed to be horizontal in the image.
	double y0 = ImageToGrid(pc->GetImageVpt(0))[1];
	double y1 = ImageToGrid(pc->GetImageVpt(1))[1];
	CHECK_EQ_TOL(y0, y1, 1e-6) << "The horizon is not horizontal in the image.";
	horizon_row = roundi((y0 + y1)/2.0);

	// Check that image is not flipped. This should be guaranteed by GetVerticalRectifier
	Vec2 floor_pt = makeVector(0, horizon_row+1);
	// Note that PosedCamera::GetImageHorizon always returns a line with positive half on the floor...
	CHECK_GT(GridToImage(floor_pt)*pc->GetImageHorizon(), 0)
		<< "The matrix returned by GetVerticalRectifier flips the image upside down!";
}

void ManhattanDP::ComputeScores() {
	for (int i = 0; i < 3; i++) {
		// Initialize
		grid_scores[i].Resize(grid_size[1], grid_size[0], 0);

		CHECK_EQ(score_func->pixel_scores[i].Rows(), pc->image_size().y);
		CHECK_EQ(score_func->pixel_scores[i].Cols(), pc->image_size().x);

		// Transform the scores according to ImageToGrid(.)
		for (int y = 0; y < pc->image_size().y; y++) {
			const float* inrow = score_func->pixel_scores[i][y];
			for (int x = 0; x < pc->image_size().x; x++) {
				Vec2I grid_pt = RoundVector(ImageToGrid(makeVector(x, y, 1.0)));
				if (grid_pt[0] >= 0 && grid_pt[0] < grid_size[0] &&
					grid_pt[1] >= 0 && grid_pt[1] < grid_size[1]) {
					grid_scores[i][ grid_pt[1] ][ grid_pt[0] ] += inrow[x];
				}
			}
		}

		// Compute the integral-col image
		integ_scores[i].Compute(grid_scores[i]);
	}
}

void ManhattanDP::ComputeOppositeRows() {
	opp_rows.Resize(grid_size[1], grid_size[0]);
	Mat3 grid_floorToCeil = imageToGrid * floorToCeil * gridToImage;  // floorToCeil in grid coords
	Mat3 grid_ceilToFloor = LU<>(grid_floorToCeil).get_inverse();
	for (int y = 0; y < grid_size[1]; y++) {
		const Mat3& m = y < horizon_row ? grid_ceilToFloor : grid_floorToCeil;
		for (int x = 0; x < grid_size[0]; x++) {
			Vec2 opp_grid_pos = project(m * makeVector(x,y,1.0));
			// TODO: is Clamp appropriate here??
			opp_rows[y][x] = /*Clamp<int>(*/opp_grid_pos[1]/*, 0, grid_size[1]-1)*/;
		}
	}
}

const DPSolution& ManhattanDP::Solve(const DPState& state) {
	cache_lookups++;
	max_depth = max(max_depth, cur_depth);

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
		if (next_row != horizon_row && next_row >= 0 && next_row < grid_size[1]) {
			DPState next = state;
			next.row = next_row;
			best.ReplaceIfSuperior(Solve(next), next);
		}

	} else if (state.dir == DPState::DIR_OUT) {
		DPState next = state;
		next.dir = DPState::DIR_IN;
		next.remaining = state.remaining-1;

		double delta_score = 0.0;
		double vpt_x = vpt_cols[state.axis];

		double m = (state.row-horizon_row)/(state.col-vpt_x);
		double c = horizon_row - m*vpt_x;

		//horiz_len_hist.Add(state.col-1); // for profiling
		for (next.col = state.col-1; next.col >= 0; next.col--) {
			// Check that we don't cross the vpt
			if (abs(vpt_x - next.col) < 1.0) break;

			// Compute the new row
			double next_y = m*next.col + c;
			next.row = roundi(next_y);

			// Check bounds.
			if (next.row < 0 || next.row >= grid_size[1]) continue;

			// Compute the row of the opposite face (floor <-> ceiling)
			// TODO: ComputeOppositeRows clamps the values in opp_rows, review
			// whether this is appropriate.
			/*int opp_row = [next.row][next.col];

			// Compute the score for this segment
			// TODO: this is the cost function: factor it out
			int wall_orient = 1-next.axis;  // labels refer to normal direction rather than vpt index
			int r0 = min(next.row, opp_row);
			int r1 = max(next.row, opp_row);
			score_delta += integ_orients.Count(vert_axis, next.col, 0, r0)
							+ integ_orients.Count(wall_orient, next.col, r0, r1)
							+ integ_orients.Count(vert_axis, next.col, r1, grid_orients.Rows()-1);*/

			// Compute cost
			delta_score += MarginalWallScore(next.row, next.col, next.axis);

			// Recurse
			best.ReplaceIfSuperior(Solve(next), next, delta_score);

			// Compute the error associated with jumping to the nearest (integer-valued) pixel
			double jump_error = abs(next.row - next_y);
			double dist = abs(next.row-state.row)+abs(next.col-state.col);  // L1 norm for efficiency
			double rel_jump_error = jump_error / dist;

			// If the error is sufficiently small then allow the line to
			// continue with a slight "kink". This approximation
			// reduces overall complexity from O( W*H*(W+H) ) to O(W*H)
			if (rel_jump_error < jump_thresh) {
				horiz_cutoff_hist.Add(state.col - next.col);
				// we just continue from this point -- don't add an intersection
				next.dir = DPState::DIR_OUT;
				next.remaining = state.remaining;
				best.ReplaceIfSuperior(Solve(next), next, delta_score);
				// The above recursion has already (approximately)
				// considered all further points along the line so there is
				// no need to continue. Note that we break here
				// regardless of whether this solution replaced the
				// best so far because if this solution did not replace
				// the best so far then nothing along this line will
				// do so.
				break;
			}

		}
	}

	return best;
}

double ManhattanDP::MarginalWallScore(int row, int col, int axis) {
	// Compute the row of the opposite face (floor <-> ceiling)
	int opp_row = Clamp<int>(opp_rows[row][col], 0, grid_size[1]-1);

	// Compute the score for this segment
	int wall_orient = 1-axis;  // labels refer to normal direction rather than vpt index
	int r0 = min(row, opp_row);
	int r1 = max(row, opp_row);
	return integ_scores[kVerticalAxis].Sum(col, 0, r0-1)
         + integ_scores[wall_orient].Sum(col, r0, r1-1)
         + integ_scores[kVerticalAxis].Sum(col, r1, grid_size[1]-1);
}

void ManhattanDP::ComputeBacktrack() {
	soln_walls.clear();
	soln_grid_walls.clear();
	soln_orients.Resize(pc->image_size().y, pc->image_size().x, kVerticalAxis);

	// Trace the solution
	const DPState* cur = &solution.src;
	const DPState* out = NULL;
	do {
		full_backtrack.push_back(cur);

		if (cur->dir == DPState::DIR_IN && out != NULL) {
			Vec2 tl = makeVector(cur->col, cur->row);  // check this! was: cur->position()
			Vec2 tr = makeVector(out->col, out->row);  // check this! was: out->position
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
	int occl_vpt_col = vpt_cols[occl_axis];
	int occl_vpt_side = occl_vpt_col < col ? -1 : 1;
	int opp_vpt_col = vpt_cols[1-occl_axis];  // irrespective of whether left_axis=right_axis!
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
		// Draw a quad
		PixelRGB<byte> color = bc.Next();
		for (int i = 0; i < 4; i++) {
			DrawLineClipped(canvas, wall.poly.edge(i), color);
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
                                       const ManhattanDP::ScoreFunction& s) {
	input_image = &image;
	TIMED("Complete DP") dp.Compute(s, image.pc(), floorToCeil);
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
	ImageRGB<byte> soln_canvas;
	ImageCopy(input_image->rgb, soln_canvas);
	DrawOrientations(dp.soln_orients, soln_canvas, 0.35);
	WriteImage(path, soln_canvas);
}

void ManhattanDPReconstructor::OutputGridViz(const string& path) {
	ImageRGB<byte> grid_canvas;
	dp.DrawGridSolution(grid_canvas);
	WriteImage(path, grid_canvas);
}

void ManhattanDPReconstructor::OutputOppRowViz(const string& path) {
	FileCanvas canvas(path, input_image->rgb);
	for (int y = 5; y < input_image->ny(); y += 100) {
		for (int x = 5; x < input_image->nx(); x += 100) {
			Vec3 v = makeVector(x,y,1.0);
			Vec2I grid = RoundVector(dp.ImageToGrid(v));
			int opp_row = dp.opp_rows[ grid[1] ][ grid[0] ];
			Vec2 opp = project(dp.GridToImage(makeVector(grid[0], opp_row)));

			bool ceil = grid[1] < dp.horizon_row;
			PixelRGB<byte> color = ceil ? Colors::blue() : Colors::green();

			canvas.DrawDot(project(v), 5, Colors::white());
			canvas.DrawDot(project(v), 4, color);
			canvas.DrawDot(opp, 2, color);
			canvas.StrokeLine(project(v), opp, color);
		}
	}

	// Draw the horizon line
	Vec2 horizon_l = project(dp.GridToImage(makeVector(0, dp.horizon_row)));
	Vec2 horizon_r = project(dp.GridToImage(makeVector(input_image->nx(), dp.horizon_row)));
	canvas.StrokeLine(horizon_l, horizon_r, Colors::white());
}

void LineSweepDPCost::OutputOrientViz(const string& path) {
	ImageRGB<byte> orient_canvas;
	ImageCopy(input_image->rgb, orient_canvas);
	line_sweeper.DrawOrientViz(orient_canvas);
	line_detector.DrawSegments(orient_canvas);
	WriteImage(path, orient_canvas);
}

void LineSweepDPCost::OutputLineViz(const string& path) {
	FileCanvas canvas(path, asToon(input_image->sz()));
	canvas.DrawImage(input_image->rgb);
	canvas.SetLineWidth(3.0);
	for (int i = 0; i < 3; i++) {
		BOOST_FOREACH(const LineDetection& det, line_detector.detections[i]) {
			canvas.StrokeLine(det.seg, Colors::primary(i));
		}
	}
}



void LineSweepDPCost::Compute(const PosedImage& image) {
	input_image = &image;

	// Detect lines and guess initial orientation
	TIMED("Detect lines") line_detector.Compute(image);
	TIMED("Estimate orientations")
		line_sweeper.Compute(image, line_detector.detections);

	// Convert the line sweeper labels to a score matrix
	scorefunc.Resize(image.nx(), image.ny());
	for (int y = 0; y < image.ny(); y++) {
		const int* inrow = line_sweeper.orient_map[y];
		for (int i = 0; i < 3; i++) {
			float* outrow = scorefunc.pixel_scores[i][y];
			for (int x = 0; x < image.nx(); x++) {
				outrow[x] = (inrow[x] == i ? 1.0 : 0.0);
			}
		}
	}
}

}


/*
void ManhattanDPReconstructor::GetAuxOrients(const PosedCamera& aux,
                                             double zfloor,
                                             MatI& aux_orients) {
	aux_orients.Resize(aux.image_size().y, aux.im_size().x);
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
