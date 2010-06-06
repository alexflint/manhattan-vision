#include <tr1/unordered_map>

#include <LU.h>

#include "manhattan_dp.h"
#include "common_types.h"
#include "map.h"
#include "camera.h"
#include "timer.h"
#include "floor_ceil_map.h"
#include "clipping.h"

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

	void ManhattanDP::Compute(const MatI& orients, const PosedCamera& cam) {
		pc = &cam;
		input_orients = &orients;
		input_size = makeVector(orients.Cols(), orients.Rows());

		ComputeGridWarp();  // note this _must_ come before everything else
		TIMED("Warp orients") ComputeOrients();
		ComputeOppositeRows();

		// Locate the vanishing points in grid coordinates
		for (int i = 0; i < 2; i++) {
			Vector<2> canon_vpt = project(pc->RetToIm(H_canon * pc->GetRetinaVpt(i)));
			horiz_vpts[i] = canon_vpt/grid_scaling + grid_offset;
		}

		// Note that H_canon breaks the orthogonality of the vanishing
		// points, so the horzon is not guaranteed to be in the middle of
		// the image even though the vertical vanishing point is
		// guaranteed to be at vertical infinity. The horizon is, however,
		// guaranteed to be horizontal in the image.
		CHECK_LE(abs(horiz_vpts[0][1] - horiz_vpts[1][1]), 1e-6)
			<< "The horizon should be horizontal in the image. Perhaps H_canon is wrong?"
			<< "the vpts are: " << horiz_vpts[0] << " and " << horiz_vpts[1];

		// Calculate the row the horizon is in
		horizon_row = roundi(horiz_vpts[0][1]);

		// Reset the profiling histograms
		horiz_len_hist.Clear();
		horiz_cutoff_hist.Clear();

		// Reset the cache
		cache_lookups = 0;
		cache_hits = 0;

		// Reset the cache
		cache.reset(grid_size, max_corners);

		// Guess a good number of buckets to use
		//cache.clear();
		//cache.rehash(orient_map.Rows() * orient_map.Cols() * max_corners * 10);

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

		//const Histogram<int>& len_hist = horiz_len_hist.Eval(10);
		//TITLED("len_hist") len_hist.RenderText();
		//len_hist.OutputViz("out/len_hist.png");
		//TITLED("Side-by-side")
		//Histogram<int>::RenderTextSbs(len_hist, cutoff_hist);*/
	}

	// Populate H_canon and H_canon_inv with appropriate warps
	void ManhattanDP::ComputeGridWarp() {
		// Build a rotation to move the vertical vanishing point to infinity.
		// R must satisfy:
		//   (1) R*up = [0,1,0]
		//   (2) R is as close to the identity as possible
		//        (so that the other vpts are minimally affected)
		Vector<3> up = pc->GetRetinaVpt(2);
		Matrix<3> R_up;
		R_up[0] = up ^ GetAxis<3>(2);
		R_up[1] = up;
		R_up[2] = R_up[0] ^ R_up[1];

		// T_centre projects the centre of the original image projects to
		// the centre of the new image
		Matrix<3> T_centre = Identity;
		T_centre.slice<0,2,2,1>() = -project(R_up*makeVector(0,0,1)).as_col();

		// Compute the warping homographies to transform the image so that
		// vertical in the world is vertical in the image.
		H_canon = T_centre * R_up;

		// Compute scaling to keep all corners within bounds
		double scale = 1.0;
		const Bounds2D<>& ret_bounds = pc->camera.ret_bounds();
		Polygon<4> ret_perimeter = ret_bounds.GetPolygon();
		for (int i = 0; i < 4; i++) {
			double canon_x = project(H_canon * ret_perimeter.verts[i])[0];
			// this works because the retina is centred at zero...
			scale = max(scale, canon_x/ret_bounds.left);
			scale = max(scale, canon_x/ret_bounds.right);
		}
		DiagonalMatrix<3> M_scale(makeVector(1.0/scale, 1.0/scale, 1.0));

		// Since H_canon operates on homogeneous coordinates we must not
		// apply the scaling to the third row
		H_canon = H_canon * M_scale;
		H_canon_inv = LU<>(H_canon).get_inverse();

		// Compute offsets and scaling applied after the canonical warp
		grid_scaling = grid_scale_factor;
		grid_offset = makeVector(0, input_size[1] * grid_offset_factor/grid_scaling);
		grid_size = input_size/grid_scaling + grid_offset*2;
	}

	// Populate integ_orients with warped orientation data
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
					Vector<3> ret_pt = pc->ImToRet(makeVector(x, y, 1.0));
					Vector<3> canon_im_pt = pc->RetToIm(H_canon * ret_pt);
					Vector<2,int> grid_pt = project(canon_im_pt)/grid_scaling + grid_offset;
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
		for (int y = 0; y < opp_rows.Rows(); y++) {
			for (int x = 0; x < opp_rows.Cols(); x++) {
				/*Vector<2> pos = makeVector(x,y);
				Vector<3> canon_im_pt = unproject((pos-grid_offset) * grid_scaling);
				Vector<3> im_pt = pc->RetToIm(H_canon_inv * pc->ImToRet(canon_im_pt));
				Vector<3> opp_im_pt = fcmap.TransferIm(im_pt);
				Vector<2> opp_grid_pos = project(H_canon*opp_im_pt)/grid_scaling + grid_offset;*/

				Vector<2> opp_grid_pos = ImageToGrid(fcmap.TransferIm(GridToImage(makeVector(x,y))));
				opp_rows[y][x] = Clamp(opp_grid_pos[1], 0, orient_map.Rows()-1);
			}
		}
	}

	// The caching wrapper for the DP
	const DPSolution& ManhattanDP::Solve(const DPState& state) {
		cache_lookups++;
		max_depth = max(max_depth, cur_depth);

		// unordered_map::insert actually does a lookup first and returns
		// an existing element if there is one, or returns an iterator
		// pointing to a newly inserted element if not.
		//pair<Cache::iterator,bool> res = cache.insert(state);
		Cache::iterator it = cache.find(state);
		if (it == cache.end()) {
			cur_depth++;
			const DPSolution& soln = (cache[state] = Solve_Impl(state));  // single equals sign intended!
			cur_depth--;
			return soln;
		} else {
			// The key was already in the map, return the precomputed value
			cache_hits++;
			//return it->second;
			return *it;
		}
	}

	// The DP implementation
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
				if (CanMove(state, next)) {
					best.ReplaceIfSuperior(Solve(next), next);
				}

				// Try going down from here
				next.dir = DPState::DIR_DOWN;
				if (CanMove(state, next)) {
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
			const Vector<2>& vpt = horiz_vpts[state.axis];

			double m = (state.row-vpt[1])/(state.col-vpt[0]);
			double c = vpt[1] - m*vpt[0];

			//horiz_len_hist.Add(state.col-1); // for profiling
			for (next.col = state.col-1; next.col >= 0; next.col--) {
				// Check that we don't cross the vpt
				if (abs(vpt[0] - next.col) < 1.0) break;

				// Compute the row
				double next_y = m*next.col + c;
				next.row = roundi(next_y);
				double error = abs(next.row - next_y);

				// Here we use the L1 norm for efficiency
				double rel_error = error / (abs(next.row-state.row)+abs(next.col-state.col));

				// Check bounds. TODO: relax this
				if (next.row < 0 || next.row >= orient_map.Rows()) continue;

				// TODO: don't cross the vanishing point's column

				// Compute the row of the opposite face (floor <-> ceiling)
				int opp_row = Clamp<int>(opp_rows[next.row][next.col], 0, orient_map.Rows()-1);
				// TODO: what if opp_row is outside the bounds of orient_map

				// Compute the score for this segment
				int r0 = min(next.row, opp_row);
				int r1 = max(next.row, opp_row);
				score_delta += integ_orients.Count(vert_axis, next.col, 0, r0)
					+ integ_orients.Count(next.axis, next.col, r0, r1)
					+ integ_orients.Count(vert_axis, next.col, r1, orient_map.Rows()-1);

				// Recurse
				best.ReplaceIfSuperior(Solve(next), next, score_delta);

				// If the error is sufficiently small then allow the line to
				// continue with a slight aberation. This approximation
				// reduces complexity from O( W*H*(W+H) ) to O(W*H)
				if (rel_error < jump_thresh) {
					horiz_cutoff_hist.Add(state.col - next.col);
					// we just continue from this point -- don't add an intersection
					next.dir = DPState::DIR_OUT;
					next.remaining = state.remaining;
					best.ReplaceIfSuperior(Solve(next), next, score_delta);
					break;  // This recursion has already (approximately)
					        // considered all further points along the line so
					        // no need to continue. Note that we break here
					        // regardless of whether this solution replaced the
					        // best so far. If this solution did _not_ replace
					        // the best so far then nothing along this line will
					        // do so.
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
				Vector<2> tl = cur->position();
				Vector<2> tr = out->position();
				Vector<2> bl = makeVector(tl[0], opp_rows[ cur->row ][ cur->col ]);
				Vector<2> br = makeVector(tr[0], opp_rows[ out->row ][ out->col ]);

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

				vector<Vector<3> > poly;
				copy_all_into(soln_walls.back().poly.verts, poly);
				FillPolygonFast(poly, soln_orients, out->axis);

				abbrev_backtrack.push_back(cur);
				out = NULL;
			} else if (cur->dir == DPState::DIR_OUT && out == NULL) {
				abbrev_backtrack.push_back(cur);
				out = cur;
			}

			cur = &cache[*cur].src;
		} while (*cur != DPState::none);
	}


	Vector<3> ManhattanDP::GridToImage(const Vector<2>& grid_x) {
		static const Vector<2> half_pixel = makeVector(0.5, 0.5);
		Vector<3> canon_x = unproject((grid_x-grid_offset+half_pixel) * grid_scaling);
		return pc->RetToIm(H_canon_inv * pc->ImToRet(canon_x));
	}

	Vector<2,int> ManhattanDP::ImageToGrid(const Vector<3>& im_x) {
		Vector<3> canon_x = pc->RetToIm(H_canon * pc->ImToRet(im_x));
		return project(canon_x)/grid_scaling + grid_offset;
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

		// is the opposite vpt between the div and the occluding vpt
		bool opp_vpt_between =
			opp_vpt_side == occl_vpt_side &&
			abs(col-opp_vpt_col) < abs(col-occl_vpt_col);

		// the occlusion is valid iff occl_vpt_behind == opp_vpt_between
		return occl_vpt_behind == opp_vpt_between;
	}

	// Determine whether we can move between two nodes in the DP based
	// on occlusion constraints.
	bool ManhattanDP::CanMove(const DPState& cur, const DPState& next) {
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






	void ManhattanReconstruction::Compute(const PosedImage& pim,
																				const proto::TruthedMap& tru_map) {
		// Identify lines
		TIMED("Detect lines") WITHOUT_DLOG line_detector.Compute(pim);

		// Compute line sweeps
		TIMED("Estimate orientations") labeller.Compute(pim, line_detector.detections);

		// The line sweeper uses labels that refer to the normal direction
		// of the wall, whereas ManhattanDP uses the horizontal tangent
		// direction. This corresponds to swapping 0s and 1s since the
		// vertical vanishing label is always 2.
		for (int y = 0; y < pim.ny(); y++) {
			int* row = labeller.orient_map[y];
			for (int x = 0; x < pim.nx(); x++) {
				// note that row[x] is one of -1,0,1,2
				if (row[x] == 1) {
					row[x] = 0;
				} else if (row[x] == 0) {
					row[x] = 1;
				}
			}
		}

		// Project a point on the floor and the ceiling into the image.
		// (TODO: do this with a line detector)
		Vector<3> ref_pt = pim.pc.invpose * (GetAxis<3>(2) * 10.0);
		Vector<3> floor_pt = ref_pt;
		floor_pt[2] = tru_map.floorplan().zfloor();
		Vector<3> ceil_pt = ref_pt;
		ceil_pt[2] = tru_map.floorplan().zceil();
		Vector<3> ret_floor_pt = atretina(pim.pc.WorldToRet(floor_pt));
		Vector<3> ret_ceil_pt = atretina(pim.pc.WorldToRet(ceil_pt));
		dp.fcmap.Compute(ret_floor_pt, ret_ceil_pt, pim.pc);

		// Run the DP
		TIMED("Complete DP") dp.Compute(labeller.orient_map, pim.pc);

		// Report statistics
		/*INDENTED {
			DLOG << "nodes evaluated: " << dp.cache_lookups - dp.cache_hits;
			DREPORT(dp.cache_lookups);
			DREPORT(dp.cache_hits);
			//DREPORT(dp.cache.load_factor());
			//DREPORT(dp.cache.bucket_count());
			DREPORT(dp.max_depth);
			}*/
	}

	void ManhattanReconstruction::ReportBacktrack() {
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
}
