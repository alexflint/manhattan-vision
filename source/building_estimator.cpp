#include <VW/Image/imagecopy.tpp>

#include "building_estimator.h"
#include "common_types.h"
#include "clipping.h"
#include "geom_utils.h"
#include "image_utils.h"
#include "bld_helpers.h"
#include "timer.h"

// TODO: remove these when debugging done
#include "viewer3d.h"
#include "widget3d.h"
#include "map_widgets.h"

#include "range_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	lazyvar<int> gvMaxCorners("ManhattanRecovery.MaxCorners");
	lazyvar<double> gvOcclThresh("ManhattanRecovery.CnrOcclusionThresh");
	lazyvar<double> gvMinCornerMargin("ManhattanRecovery.MinCornerMargin");
	lazyvar<int> gvOrientRes("ManhattanRecovery.OrientRes");

	void ClipToFront(Vector<3>& a, Vector<3>& b) {
		const double kClipDist = 0.1;
		if (b[2] < 0 && a[2] > 0) {
			double t = (a[2] - kClipDist) / (a[2] - b[2]);
			b = t*b + (1-t)*a;
		} else if (a[2] < 0 && b[2] > 0) {
			double t = (b[2] - kClipDist) / (b[2] - a[2]);
			a = t*a + (1-t)*b;
		}
	}

	ManhattanEdge::ManhattanEdge() : id(-1) {
	}

	ManhattanEdge::ManhattanEdge(const Vector<3>& a,
															 const Vector<3>& b,
															 int ax,
															 int i)
		: start(pve_unit(a)), end(pve_unit(b)), eqn(start^end), axis(ax), id(i) {
	}


	bool ManhattanEdge::Contains(const Vector<3>& p) const {
		// This notion of distance only makes sense for homogeneous
		// coordinates when z coordinates are positive
		Vector<3> x = pve_unit(p);
		return min(x*start, x*end) > start*end;
	}

	bool ManhattanBuilding::ContainsEdge(int id) const {
		return edge_ids.find(id) != edge_ids.end();
	}





	ManhattanRecovery::ManhattanRecovery()
		:	max_corners(*gvMaxCorners) {
	}

	void ManhattanRecovery::Compute(const PosedCamera& pcam,
																	const vector<ManhattanEdge> edges[],
																	const MatI& orients) {
		pc = &pcam;
		success = true;
		
		int res = *gvOrientRes;
		MatI est_orients;
		DownsampleOrients(orients, est_orients, makeVector(res, res));

		TIMED("Initialize hypotheses") WITHOUT_DLOG Initialize(edges);
		if (hypotheses.empty()) {
			DLOG << "Failed to generate initial hypotheses, aborting.";
			success = false;
			return;
		}

		TIMED("Enumerate hypotheses") INDENTED Enumerate();
		DLOG << "Generated " << hypotheses.size() << " hypotheses";

		TIMED("Evaluate hypotheses") EvaluateHypotheses(est_orients);
	}



	void ManhattanRecovery::Initialize(const vector<ManhattanEdge> edges[]) {
		hypotheses.clear();

		// Compute size of a pixel
		Vector<2> im_c = pc->RetToIm(makeVector(0.0, 0.0));
		Vector<2> im_d = pc->RetToIm(makeVector(1.0, 1.0));
		px_diam = 1/norm(im_c-im_d);

		// Cache the vanishing points for efficiency
		// vert_axis is the vertical direction
		vert_axis = 0;
		for (int i = 0; i < 3; i++) {
			vpts[i] = pc->GetRetinaVpt(i);
			if (abs(vpts[i][1]) > abs(vpts[vert_axis][1])) {
				vert_axis = i;
			}
		}
		DREPORT(vert_axis);

		h1_axis = (vert_axis+1)%3;
		h2_axis = (vert_axis+2)%3;
		vert_vpt = vpts[vert_axis];
		h1_vpt = vpts[(vert_axis+1)%3];
		h2_vpt = vpts[(vert_axis+2)%3];

		// Init axis is the axis we'll use to generate initial hypotheses
		if (abs(h1_vpt[0]) > abs(h2_vpt[0])) {
			init_axis = h1_axis;
		} else {
			init_axis = h2_axis;
		}
		DREPORT(init_axis);

		// Ensure the horizon line has its positive side at the top of the image
		horizon = h1_vpt ^ h2_vpt;
		if (horizon[1] < 0) horizon = -horizon;

		// Find the left/right bounds
		// TODO: do this from the image bounds instead
		double xmin = INFINITY, xmax = -INFINITY;
		for (int i = 0; i < 3; i++) {
			BOOST_FOREACH(const ManhattanEdge& e, edges[i]) {
				double x1 = e.start[0] / e.start[2];
				double x2 = e.end[0] / e.end[2];
				if (x1 > x2) swap(x1, x2);
				if (x1 < xmin) xmin = x1;
				if (x2 > xmax) xmax = x2;
			}
		}
		Vector<3> left_div = DivVec(unit(vert_vpt ^ makeVector(xmin-1, 0, 1)));
		Vector<3> right_div = DivVec(unit(vert_vpt ^ makeVector(xmax+1, 0, 1)));

		// Identify edges above and below horizon
		for (int a = vert_axis+1; a <= vert_axis+2; a++) {
			int axis = a%3;
			ceil_edges[axis].clear();
			floor_edges[axis].clear();

			const vector<ManhattanEdge>& es = edges[axis];
			BOOST_FOREACH(const ManhattanEdge& e, es) {
				if (AboveHorizon(e.start) && AboveHorizon(e.end)) {
					ceil_edges[axis].push_back(&e);
				} else if (BelowHorizon(e.start) && BelowHorizon(e.end)) {
					floor_edges[axis].push_back(&e);
				}
				// other edges cross the horizon -- ignore them
			}
		}

		// Make a list of vertical edges
		vert_edges.clear();
		BOOST_FOREACH(const ManhattanEdge& e, edges[vert_axis]) {
			vert_edges.push_back(&e);
		}

		// Enumerate initial candidates. We have already decided which axis they will come from
		DREPORT(ceil_edges[init_axis].size());
		DREPORT(floor_edges[init_axis].size());
		COUNTED_FOREACH(int ii, const ManhattanEdge* ceil_e, ceil_edges[init_axis]) {
			Vector<3> div1 = ceil_e->start ^ vert_vpt;
			Vector<3> div2 = ceil_e->end ^ vert_vpt;
			COUNTED_FOREACH(int jj, const ManhattanEdge* floor_e, floor_edges[init_axis]) {
				Vector<3> div3 = floor_e->start ^ vert_vpt;
				Vector<3> div4 = floor_e->end ^ vert_vpt;
				Vector<3> p1 = div1 ^ floor_e->eqn;
				Vector<3> p2 = div2 ^ floor_e->eqn;
				Vector<3> p3 = div3 ^ ceil_e->eqn;
				Vector<3> p4 = div4 ^ ceil_e->eqn;
				if (ceil_e->Contains(p3) ||	ceil_e->Contains(p4) ||
						floor_e->Contains(p1) || floor_e->Contains(p2)) {
					// There is overlap, we can create an initial hypothesis
					ManhattanCorner left;
					left.div_eqn = left_div;
					left.right_ceil = left_div ^ ceil_e->eqn;
					left.right_floor = left_div ^ floor_e->eqn;
					left.right_axis = init_axis;
					left.leftmost = true;

					ManhattanCorner right;
					right.div_eqn = right_div;
					right.left_ceil = right_div ^ ceil_e->eqn;
					right.left_floor = right_div ^ floor_e->eqn;
					right.left_axis = init_axis;
					right.rightmost = true;

					ManhattanBuilding* bld = new ManhattanBuilding;
					bld->cnrs.push_back(left);
					bld->cnrs.push_back(right);
					hypotheses.push_back(bld);  // memory now managed by the ptr_vector
				}
			}
		}
	}


	void ManhattanRecovery::Enumerate() {
		CHECK(!hypotheses.empty());
		DLOG << "Initialized with " << hypotheses.size() << " hypotheses";
		int start = 0;
		for (int i = 0; i < max_corners; i++) {
			TITLED("Branch " + lexical_cast<string>(i)) {
				int end = hypotheses.size();
				int nhoriz = 0;
				int nvert = 0;
				WITHOUT_DLOG 
				for (int j = start; j < end; j++) {
					nvert += VertBranchFrom(hypotheses[j], hypotheses);
					nhoriz += HorizBranchFrom(hypotheses[j], hypotheses);
				}
				DLOG << "Produced " << (hypotheses.size()-end) << " new structures";
				start = end;
			}
		}
	}

	void ManhattanRecovery::EvaluateHypotheses(const MatI& est_orients) {
		// Compute scores for each hypothesis
		scores.resize(hypotheses.size());
		fill_all(scores, 0);
		ProgressReporter score_pro(hypotheses.size(), "Scoring buildings");
		ParallelPartition(hypotheses.size(),
											bind(&ManhattanRecovery::EvaluateHypotheses,
													 this, ref(est_orients), _1, _2, ref(score_pro)));

		// Find the best building
		soln_index = max_index(scores.begin(), scores.end());
		soln_score = scores[soln_index];
		soln = &hypotheses[soln_index];

		// Predict orientations
		PredictImOrientations(*soln, soln_orients);
	}

	void ManhattanRecovery::EvaluateHypotheses(const MatI& est_orients, int first, int last,
																						 ProgressReporter& pr) {
		MatI predicted(est_orients.Rows(), est_orients.Cols());
		for (int i = first; i <= last; i++) {
			PredictOrientations(hypotheses[i], predicted);
			scores[i] = ScorePrediction(predicted, est_orients);
			pr.Increment();  // ProgressReporter is thread-safe
		}
	}




	int ManhattanRecovery::OtherHorizAxis(int a) const {
		return a == h1_axis ? h2_axis : h1_axis;
	}

	bool ManhattanRecovery::AboveHorizon(const Vector<3>& v) const {
		// the horizon always has y>0
		return PointSign(v, horizon) < 0;
	}

	bool ManhattanRecovery::BelowHorizon(const Vector<3>& v) const {
		// the horizon always has y>0
		return PointSign(v, horizon) > 0;
	}

	bool ManhattanRecovery::OcclusionValid(ManhattanCorner& cnr) {
		CHECK_LE(abs(cnr.occl_side), 1);  // check for uninitialized

		if (cnr.occl_side == 0) {
			// No occlusion, nothing to check
			return true;
		}

		// the axis of the occluding side
		int occl_axis = cnr.occl_side < 0 ? cnr.left_axis : cnr.right_axis;

		// the other horizontal axis (irrespective of cnr.left_axis, cnr.right_axis!)
		int opp_axis = (occl_axis == h1_axis) ? h2_axis : h1_axis;

		CHECK(abs(norm_sq(vpts[occl_axis]) - 1.0) < 1e-5);  // should be guaranteed of this
		CHECK(abs(norm_sq(vpts[opp_axis]) - 1.0) < 1e-5);  // should be guaranteed of this

		// which side is the associated vpt on?
		// -1 for left, 1 for right since div_eqn[0]>0
		int vpt_side = PointSign(vpts[occl_axis], cnr.div_eqn);

		// is the vpt on the same side as the occluding segment?
		bool vpt_behind = vpt_side == cnr.occl_side;
		
		// is the other vpt between the div and the vpt
		bool opp_vpt_between =
			PointSign(vpts[opp_axis], cnr.div_eqn) == vpt_side &&
			abs(cnr.div_eqn*vpts[opp_axis]) < abs(cnr.div_eqn*vpts[occl_axis]);
		
		// the occlusion is valid iff vpt_behind <=> opp_vpt_between
		return vpt_behind == opp_vpt_between;
	}
	
		

	// Update the occlusion member
	bool ManhattanRecovery::UpdateOcclusion(ManhattanCorner& cnr) {
		if (cnr.leftmost || cnr.rightmost) {
			// The bounding corners cannot be occluded and should not be
			// checked since they have meaningless floor and ceiling points
			// on their outer sides.
			cnr.occl_side = 0;
			return true;
		}

		if (!BelowHorizon(cnr.left_floor) || !BelowHorizon(cnr.right_floor) ||
				!AboveHorizon(cnr.left_ceil) || !AboveHorizon(cnr.right_ceil)) {
			DLOG << "culled because intersection on wrong side of horizon";
			return false;
		}
			

		const double kThresh = *gvOcclThresh * px_diam;
		const double kThreshSq = kThresh*kThresh;
		double floor_diff = HNormSq(cnr.left_floor, cnr.right_floor);
		double ceil_diff = HNormSq(cnr.left_ceil, cnr.right_ceil);
		if (floor_diff <= kThreshSq || ceil_diff <= kThreshSq) {
			// Below threshold, no occlusion
			cnr.occl_side = 0;
		} else {
			if (!BelowHorizon(cnr.left_floor)) {
				WITH_DLOG DREPORT(cnr.left_floor, horizon,
													cnr.left_floor*horizon,
													PointSign(cnr.left_floor, horizon));
			}
			CHECK_PRED1(BelowHorizon, cnr.left_floor);
			CHECK_PRED1(BelowHorizon, cnr.right_floor);
			// TODO: I think we should use pve_unit below since if
			// cnr.left_floor[2] or cnr.right_floor[2] is negative then the
			// inequality will be swapped
			double l_dp = unit(cnr.left_floor) * horizon;
			double r_dp = unit(cnr.right_floor) * horizon;
			cnr.occl_side = l_dp > r_dp ? -1 : 1;  // -1 means left occludes right
		}

		return true;
	}

	bool ManhattanRecovery::UpdateStripe(ManhattanCorner& left_cnr,
																			 ManhattanCorner& right_cnr,
																			 const Vector<3>& floor_line,
																			 const Vector<3>& ceil_line,
																			 int new_axis) {

		left_cnr.right_floor = left_cnr.div_eqn ^ floor_line;
		left_cnr.right_ceil = left_cnr.div_eqn ^ ceil_line;
		left_cnr.right_axis = new_axis;

		right_cnr.left_floor = right_cnr.div_eqn ^ floor_line;
		right_cnr.left_ceil = right_cnr.div_eqn ^ ceil_line;
		right_cnr.left_axis = new_axis;

		if (!UpdateOcclusion(left_cnr)) {
			return false;
		}
		if (!UpdateOcclusion(right_cnr)) {
			return false;
		}

		if (!OcclusionValid(left_cnr) || !OcclusionValid(right_cnr)) {
			DLOG << "culling due to invalid occlusion";
			return false;
		}

		return true;
	}

	// Determine the stripe containing the point p, set l_cnr and r_cnr
	// to the corners to the point's left and right respectively, and
	// return the index of the left corner. The const version takes a
	// const building and returns const iterators into it.
	int ManhattanRecovery::LocateStripe(const ManhattanBuilding& bld,
																			const Vector<3>& p,
																			ManhattanBuilding::ConstCnrIt& l_cnr,
																			ManhattanBuilding::ConstCnrIt& r_cnr) const {
		int index = -1;
		for (r_cnr = bld.cnrs.begin(); r_cnr != bld.cnrs.end(); r_cnr++) {
			if (PointSign(p, r_cnr->div_eqn) < 0) break;
			l_cnr = r_cnr;
			index++;
		}
		CHECK(r_cnr != bld.cnrs.begin());
		CHECK(r_cnr != bld.cnrs.end());
		return index;
	}

	// Determine the stripe containing the point p, set l_cnr and r_cnr
	// to the corners to the point's left and right respectively, and
	// return the index of the left corner. The non-const version takes
	// a non-const hypotheses and returns non-const iterators into it.
	int ManhattanRecovery::LocateStripe(ManhattanBuilding& bld,
																			const Vector<3>& p,
																			ManhattanBuilding::CnrIt& l_cnr,
																			ManhattanBuilding::CnrIt& r_cnr) {
		int index = -1;
		for (r_cnr = bld.cnrs.begin(); r_cnr != bld.cnrs.end(); r_cnr++) {
			if (PointSign(p, r_cnr->div_eqn) < 0) break;
			l_cnr = r_cnr;
			index++;
		}
		CHECK(r_cnr != bld.cnrs.begin());
		CHECK(r_cnr != bld.cnrs.end());
		return index;
	}

	bool ManhattanRecovery::StripeContains(const Vector<3>& left_div,
																				 const Vector<3>& right_div, 
																				 const Vector<3>& p) {
		CHECK_GE(left_div[0], 0);
		CHECK_GE(right_div[0], 0);
		return PointSign(p, left_div) > 0 && PointSign(p, right_div) < 0;
	}

	// Determine whether p is between left and right corners
	bool ManhattanRecovery::StripeContains(const ManhattanCorner& left,
																				 const ManhattanCorner& right, 
																				 const Vector<3>& p) {
		return StripeContains(left.div_eqn, right.div_eqn, p);
	}

	// Get the minimum distance from a point to the boundary of a stripe
	double ManhattanRecovery::GetStripeMargin(const ManhattanCorner& left,
																						const ManhattanCorner& right,
																						const Vector<3>& p) {
		double d1 = EuclPointLineDist(p, left.div_eqn);
		double d2 = EuclPointLineDist(p, left.div_eqn);
		return min(d1, d2);
	}

	// Returns either v or -v, whichver is a valid div_eqn (i.e. has v[0] > 0)
	Vector<3> ManhattanRecovery::DivVec(const Vector<3>& v) {
		return v[0] > 0 ? v : -v;
	}

	// Add a corner to bld. Return true if the corner is valid
	bool ManhattanRecovery::AddCorner(ManhattanBuilding& bld,
																		int edge_id,
																		const Vector<3>& div_eqn,
																		int new_axis,
																		bool update_left) {
		ManhattanBuilding::CnrIt l_cnr, r_cnr;
		Vector<3> horizon_pt = DivVec(horizon ^ div_eqn);
		LocateStripe(bld, horizon_pt, l_cnr, r_cnr);

		// Create the new corner
		ManhattanCorner new_cnr;
		new_cnr.div_eqn = DivVec(div_eqn);

		// Check that it doesn't contain the vpt
		Vector<3> ceil = l_cnr->right_ceil ^ r_cnr->left_ceil;
		Vector<3> floor = l_cnr->right_floor ^ r_cnr->left_floor;
		new_cnr.left_ceil = new_cnr.right_ceil = new_cnr.div_eqn^ceil;
		new_cnr.left_floor = new_cnr.right_floor = new_cnr.div_eqn^floor;
		new_cnr.left_axis = new_cnr.right_axis = l_cnr->right_axis;

		bld.edge_ids.insert(edge_id);

		ManhattanBuilding::CnrIt new_cnr_it = bld.cnrs.insert(r_cnr, new_cnr);
		Vector<3> new_floor = vpts[new_axis] ^ new_cnr.left_floor;
		Vector<3> new_ceil = vpts[new_axis] ^ new_cnr.left_ceil;
		
		DLOG << "updating " << (update_left ? "left" : "right") << " stripe";

		if (update_left) {
			return UpdateStripe(*l_cnr, *new_cnr_it, new_floor, new_ceil, new_axis);
		} else {
			return UpdateStripe(*new_cnr_it, *r_cnr, new_floor, new_ceil, new_axis);
		}
	}

	// Generate all building cnrss reachable by adding a single
	// horizontal edge to the specified building
	int ManhattanRecovery::HorizBranchFrom(const ManhattanBuilding& bld,
																				 ptr_vector<ManhattanBuilding>& out) {
		int count = 0;

		for (int a = 1; a <= 2; a++) {
			int new_axis = (vert_axis + a) % 3;
			for (int surf = 0; surf <= 1; surf++) {
				const vector<const ManhattanEdge*>& es = surf ?
					ceil_edges[new_axis] : floor_edges[new_axis];

				BOOST_FOREACH(const ManhattanEdge* e, es) {
					TITLE("Creating building " + lexical_cast<string>(out.size()));
					DLOG << "Considering adding horiz edge";

					// Don't add the same edge twice
					if (bld.ContainsEdge(e->id)) {
						DLOG << "culled because edge already present";
						continue;
					}

					ManhattanBuilding::ConstCnrIt l_cnr, r_cnr;
					LocateStripe(bld, e->start, l_cnr, r_cnr);

					// Check that the stripe fully contains the edge
					if (!StripeContains(l_cnr->div_eqn, r_cnr->div_eqn, e->end)) {
						// Horizontal segment spans multiple stripes
						// TODO: make this a threshold rather than a hard rule
						// TODO: maybe even remove this rule?
						DLOG << "culled because end point outside stripe";
						continue;
					}

					// Check that the stripe is generated from the opposite axis
					if (l_cnr->right_axis == e->axis) {
						DLOG << "culled because axis already matches";
						continue;
					}

					Vector<3> seg = surf ? l_cnr->right_ceil ^ r_cnr->left_ceil
						: l_cnr->right_floor ^ r_cnr->left_floor;
					Vector<3> isct = e->eqn ^ seg;
					Vector<3> new_div = DivVec(isct ^ vert_vpt);

					// Check that the intersection is within the stripe
					if (!StripeContains(l_cnr->div_eqn, r_cnr->div_eqn, isct)) {
						DLOG << "culled because intersection outside stripe";
						continue;
					}

					// Don't add the div if too close to an existing edge
					double margin = GetStripeMargin(*l_cnr, *r_cnr, isct);
					if (margin < *gvMinCornerMargin*px_diam) {
						DLOG << "culled because margin is " << margin;
						continue;
					}

					// case 1: change the stripe on the left
					if (!StripeContains(l_cnr->div_eqn, new_div, vpts[e->axis])) {
						DLOG << "Adding with change to left";
						ManhattanBuilding* new_bld = new ManhattanBuilding(bld);
						if (AddCorner(*new_bld, e->id, new_div, e->axis, true)) {
							out.push_back(new_bld);
							count++;
						}
					} else {
						DLOG << "culled left due to vpt";
					}

					// case 2: change the stripe on the right
					if (!StripeContains(new_div, r_cnr->div_eqn, vpts[e->axis])) {
						DLOG << "Adding with change to right";
						ManhattanBuilding* new_bld = new ManhattanBuilding(bld);
						if (AddCorner(*new_bld, e->id, new_div, e->axis, false)) {
							out.push_back(new_bld);
							count++;
						}
					} else {
						DLOG << "culled right due to vpt";
					}
				}
			}
		}
		return count;
	}

	// Generate all building cnrss reachable by adding a single
	// vertical edge to the specified building
	int ManhattanRecovery::VertBranchFrom(const ManhattanBuilding& bld,
																				ptr_vector<ManhattanBuilding>& out) {
		int count = 0;

		BOOST_FOREACH(const ManhattanEdge* e, vert_edges) {
			TITLE("Creating building " + lexical_cast<string>(out.size()));
			DLOG << "Considering adding vert edge";

			// Don't add the same edge twice
			if (bld.ContainsEdge(e->id)) {
				DLOG << "culled because edge already present";
				continue;
			}

			Vector<3> midp = HMidpoint(e->start, e->end);
			Vector<3> new_div = DivVec(midp ^ vert_vpt);

			// Find the stripe this edge is in
			ManhattanBuilding::ConstCnrIt l_cnr, r_cnr;
			LocateStripe(bld, midp, l_cnr, r_cnr);

			// Don't add the div if too close to an existing edge
			double margin = GetStripeMargin(*l_cnr, *r_cnr, midp);
			if (margin < *gvMinCornerMargin*px_diam) {
				DLOG << "culled because margin is " << margin;
				continue;
			}

			// Configure the new corner
			Vector<3> ceil = l_cnr->right_ceil ^ r_cnr->left_ceil;
			Vector<3> floor = l_cnr->right_floor ^ r_cnr->left_floor;

			// Don't add the corner if it crosses the existing floor or ceiling lines
			if ((Sign(ceil*e->start) == Sign(floor*e->start)) ||
					(Sign(ceil*e->end) == Sign(floor*e->end))) {
				DLOG << "culled because it crosses the floor or ceiling";
				continue;
			}

			int axis = l_cnr->right_axis;
			int new_axis = (axis == h1_axis) ? h2_axis : h1_axis;

			// case 1: change the stripe on the left
			if (StripeContains(l_cnr->div_eqn, new_div, vpts[new_axis])) {
				DLOG << "culled left due to vpt";
			} else {
				DLOG << "Adding with change to left";
				ManhattanBuilding* new_bld = new ManhattanBuilding(bld);
				if (AddCorner(*new_bld, e->id, new_div, new_axis, true)) {
					out.push_back(new_bld);
					count++;
				}
			}

			// case 2: change the stripe on the right
			if (StripeContains(new_div, r_cnr->div_eqn, vpts[new_axis])) {
				DLOG << "culled right due to vpt";
			} else {
				DLOG << "Adding with change to right";
				ManhattanBuilding* new_bld = new ManhattanBuilding(bld);
				if (AddCorner(*new_bld, e->id, new_div, new_axis, false)) {
					out.push_back(new_bld);
					count++;
				}
			}
		}
		return count;
	}




	void ManhattanRecovery::PredictOrientations(const ManhattanBuilding& bld,
																							MatI& orients) const {
		CHECK_GT(orients.Rows(), 0);
		CHECK_GT(orients.Cols(), 0);
		orients.Fill(vert_axis);
		Polygon<4> wall;
		DiagonalMatrix<3> Mscale(makeVector(1.0*orients.Cols()/pc->im_size().x,
																				1.0*orients.Rows()/pc->im_size().y,
																				1.0));
		for (ManhattanBuilding::ConstCnrIt left_cnr = bld.cnrs.begin();
				 successor(left_cnr) != bld.cnrs.end();
				 left_cnr++) {
			ManhattanBuilding::ConstCnrIt right_cnr = successor(left_cnr);
			wall.verts[0] = Mscale * pc->RetToIm(left_cnr->right_ceil);
			wall.verts[1] = Mscale * pc->RetToIm(right_cnr->left_ceil);
			wall.verts[2] = Mscale * pc->RetToIm(right_cnr->left_floor);
			wall.verts[3] = Mscale * pc->RetToIm(left_cnr->right_floor);
			FillPolygonFast(array_range(wall.verts, 4),
											orients,
											OtherHorizAxis(left_cnr->right_axis));
		}
	}

	void ManhattanRecovery::PredictGridOrientations(const ManhattanBuilding& bld,
																									MatI& orients) const {
		orients.Resize(*gvOrientRes, *gvOrientRes);
		PredictOrientations(bld, orients);
	}
	void ManhattanRecovery::PredictImOrientations(const ManhattanBuilding& bld,
																								MatI& orients) const {
		orients.Resize(pc->im_size().y, pc->im_size().x);
		PredictOrientations(bld, orients);
	}



	int ManhattanRecovery::ScorePrediction(const MatI& prediction,
																				 const MatI& orient_est) const {
		CHECK_EQ(prediction.Rows(), orient_est.Rows());
		CHECK_EQ(prediction.Cols(), orient_est.Cols());
		int score = 0;
		for (int y = 0; y < prediction.Rows(); y++) {
			for (int x = 0; x < prediction.Cols(); x++) {
				if (prediction[y][x] == orient_est[y][x]) {
					score++;
				}
			}
		}
		return score;
	}

	int ManhattanRecovery::ScoreBuilding(const ManhattanBuilding& bld,
																			 const MatI& orient_est) const {
		MatI predicted(orient_est.Cols(), orient_est.Rows());
		PredictOrientations(bld, predicted);
		return ScorePrediction(predicted, orient_est);
	}



	Vector<3> ManhattanRecovery::FloorPoint(const Vector<3>& p, double z) {
		// Note that p appear in the numerator _and_ the denominator
		// below, so linear scaling has no effect (as expected for
		// homogeneous coordinates)
		return -z*p / (p*horizon);
	}

	Vector<3> ManhattanRecovery::CeilPoint(const Vector<3>& p,
																				 const Vector<3>& floor_pt) {
		Matrix<3,2> m;
		Vector<3> u = unit(p);
		// unit() below is important because it affects the weighting of the least-squares problem
		m.slice<0,0,3,1>() = u.as_col();
		m.slice<0,1,3,1>() = -horizon.as_col();
		Vector<2> soln = SVD<3,2>(m).backsub(floor_pt);
		return soln[0] * u;
	}

	void ManhattanRecovery::TransferBuilding(const ManhattanBuilding& bld,
																					 const SE3<>& orig_pose,
																					 const SE3<>& new_pose,
																					 double floor_z,
																					 MatI& orients,
																					 const Map& map) {
		// Compute scaling (TODO: actually use this)
		DiagonalMatrix<3> Mscale(makeVector(1.0*orients.Cols()/pc->im_size().x,
																				1.0*orients.Rows()/pc->im_size().y,
																				1.0));

		// Invert the pose
		const SE3<> orig_inv = orig_pose.inverse();

		// Draw each wall segment
		//Viewer3D v;

		orients.Fill(vert_axis);
		for (ManhattanBuilding::ConstCnrIt left_cnr = bld.cnrs.begin();
				 successor(left_cnr) != bld.cnrs.end();
				 left_cnr++) {
			ManhattanBuilding::ConstCnrIt right_cnr = successor(left_cnr);

			TITLE("Next corner");
			int axis = OtherHorizAxis(left_cnr->right_axis);

			// Compute vertices in the 3D camera frame
			Vector<3> bl = FloorPoint(left_cnr->right_floor, floor_z);
			Vector<3> br = FloorPoint(right_cnr->left_floor, floor_z);
			Vector<3> tl = CeilPoint(left_cnr->right_ceil, bl);
			Vector<3> tr = CeilPoint(right_cnr->left_ceil, br);
			if (bl[2] < 0) {
				bl = -bl;
				br = -br;
				tl = -tl;
				tr = -tr;
			}
			DREPORT(project(left_cnr->right_ceil));
			DREPORT(bl,br,tl,tr);

			// Compute vertices in the 3D global frame
			Vector<3> world_tl = orig_inv * tl;
			Vector<3> world_tr = orig_inv * tr;
			Vector<3> world_bl = orig_inv * bl;
			Vector<3> world_br = orig_inv * br;

			// Project into the viewer
			/*Vector<3> floor_c = orig_inv.get_translation();
			Vector<3> ceil_c = orig_inv.get_translation();
			floor_c[2] = floor_z;
			ceil_c[2] = world_tl[2];
			QuadWidget* w;
			v.AddOwned(w = new QuadWidget(world_tl, world_tr, world_br, world_bl));
			w->color = Colors::primary(axis);
			v.AddOwned(w = new QuadWidget(world_bl, world_br, floor_c, floor_c));
			w->color = Colors::blue();
			v.AddOwned(w = new QuadWidget(world_tl, world_tr, ceil_c, ceil_c));
			w->color = Colors::white();
			v.AddOwned(new MapWidget(&map));*/

			// Compute the wall corners in the other camera
			Vector<3> ret_tl = new_pose * world_tl;
			Vector<3> ret_tr = new_pose * world_tr;
			Vector<3> ret_bl = new_pose * world_bl;
			Vector<3> ret_br = new_pose * world_br;
			ClipToFront(ret_tr, ret_tl);
			ClipToFront(ret_br, ret_bl);

			// Compute the wall corners in the other image
			Vector<3> im_tl = pc->RetToIm(ret_tl);
			Vector<3> im_tr = pc->RetToIm(ret_tr);
			Vector<3> im_bl = pc->RetToIm(ret_bl);
			Vector<3> im_br = pc->RetToIm(ret_br);

			// Build the polygon and fill
			vector<Vector<3> > poly;
			poly.push_back(im_tl);
			poly.push_back(im_bl);
			poly.push_back(im_br);
			poly.push_back(im_tr);
			FillPolygonFast(poly, orients, axis);
		}

		//v.Run();
	}

	void ManhattanRecovery::DrawBuilding(const ManhattanBuilding& bld,
																			 ImageRGB<byte>& canvas) {
		PixelRGB<byte> colors[3];
		colors[vert_axis] = Colors::primary(vert_axis);
		colors[h1_axis] = Colors::primary(h1_axis);
		colors[h2_axis] = Colors::primary(h2_axis);
		ManhattanBuilding::ConstCnrIt a = bld.cnrs.begin();
		ManhattanBuilding::ConstCnrIt b = successor(a);
		while (b != bld.cnrs.end()) {
			Vector<2> tl = project(pc->RetToIm(a->right_ceil));
			Vector<2> bl = project(pc->RetToIm(a->right_floor));
			Vector<2> tr = project(pc->RetToIm(b->left_ceil));
			Vector<2> br = project(pc->RetToIm(b->left_floor));
			PixelRGB<byte> floor_color = colors[a->right_axis];
			floor_color.r /= 2;
			floor_color.g /= 2;
			floor_color.b /= 2;
			DrawLineClipped(canvas, tl, bl, colors[vert_axis]);
			DrawLineClipped(canvas, tr, br, colors[vert_axis]);
			DrawLineClipped(canvas, tl, tr, colors[a->right_axis]);
			DrawLineClipped(canvas, bl, br, floor_color);
			a++;
			b++;
		}
	}

	void ManhattanRecovery::DrawPrediction(const ManhattanBuilding& bld,
																				 ImageRGB<byte>& canvas) {
		MatI predicted;
		PredictImOrientations(bld, predicted);
		DrawOrientations(predicted, canvas);
	}

	void ManhattanRecovery::DrawOrientations(const MatI& orients,
																					 ImageRGB<byte>& canvas) {
		ResizeImage(canvas, pc->im_size());
		for (int y = 0; y < pc->im_size().y; y++) {
			PixelRGB<byte>* row = canvas[y];
			const int* orient_row = orients[y*orients.Rows()/pc->im_size().y];
			for (int x = 0; x < pc->im_size().x; x++) {
				int orient = orient_row[x*orients.Cols()/pc->im_size().x];
				if (orient == -1) {
					row[x] = PixelRGB<byte>(255,255,255);
				} else {
					row[x] = Colors::primary(orient);
				}
			}
		}
	}

	void ManhattanRecovery::WriteBuilding(const ManhattanBuilding& bld,
																				const string& filename) {
		ofstream output(filename.c_str());
		output << "num_corners: " << bld.cnrs.size() << endl;
		output << "edges: " << iowrap(bld.edge_ids) << endl;
		COUNTED_FOREACH (int i, const ManhattanCorner& cnr, bld.cnrs) {
			output << "corner " << i << endl;
			output << "  left_axis: " << cnr.left_axis << endl;
			output << "  right_axis: " << cnr.right_axis << endl;
			output << "  occl_side: " << cnr.occl_side << endl;
			output << "  left_floor: " << pc->RetToIm(cnr.left_floor) << endl;
			output << "  right_floor: " << pc->RetToIm(cnr.right_floor) << endl;
			output << "  left_ceil: " << pc->RetToIm(cnr.left_ceil) << endl;
			output << "  right_ceil: " << pc->RetToIm(cnr.right_ceil) << endl;
		}
		output.close();
	}			

	void ManhattanRecovery::OutputBuildingViz(const ImageRGB<byte>& image,
																						const ManhattanBuilding& bld,
																						const string& filename) {
		ImageRGB<byte> canvas;
		ImageCopy(image, canvas);
		DrawBuilding(bld, canvas);
		WriteImage(filename, canvas);
	}

	void ManhattanRecovery::OutputPredictionViz(const ManhattanBuilding& bld,
																							const string& filename) {
		ImageRGB<byte> canvas;
		DrawPrediction(bld, canvas);
		WriteImage(filename, canvas);
	}

	void ManhattanRecovery::OutputAllViz(const ImageRGB<byte>& image,
																			 const ManhattanBuilding& bld,
																			 const string& basename) {
		OutputBuildingViz(image, bld, basename+"bld.png");
		OutputPredictionViz(bld, basename+"prediction.png");
		WriteBuilding(bld, basename+"info.txt");
	}
}
