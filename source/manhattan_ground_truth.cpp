#include "manhattan_ground_truth.h"

#include <queue>

#include <TooN/LU.h>

#include "common_types.h"
#include "floorplan_renderer.h"
#include "camera.h"
#include "map.pb.h"
#include "clipping.h"
#include "geom_utils.h"

#include "vector_utils.tpp"
#include "numeric_utils.tpp"

namespace indoor_context {
	using namespace toon;

	// Some tools for CountWalls below...
	namespace {
		struct Interval {
			double lo, hi;
			Interval() {}
			Interval(double a, double b) : lo(min(a,b)), hi(max(a,b)) { }
			bool empty() const { return lo==hi; }
		};

		ostream& operator<<(ostream& o, const Interval& iv) {
			return o << "{"<<iv.lo<<".."<<iv.hi<<"}";
		}

		Interval Intersect(const Interval& a, const Interval& b) {
			return Interval(Clamp<double>(a.lo, b.lo, b.hi), Clamp<double>(a.hi, b.lo, b.hi));
		}

		struct Token {
			double x;
			int index;
			bool activate;
			Token() { }
			Token(double xx, int i, bool a) : x(xx), index(i), activate(a) { }
			bool operator<(const Token& t) const {
				return x<t.x;
			}
		};

		struct TableComp {
			const MatI& table;
			TableComp(const MatI& t) : table(t) { }
			bool operator()(const int& a, const int& b) const {
				return table[a][b] > 0;
			}
		};
	}




	ManhattanGroundTruth::ManhattanGroundTruth() {
	}

	ManhattanGroundTruth::ManhattanGroundTruth(const proto::FloorPlan& gt_floorplan,
																						 const PosedCamera& camera) {
		Compute(gt_floorplan, camera);
	}

	void ManhattanGroundTruth::Compute(const proto::FloorPlan& gt_floorplan,
																		 const PosedCamera& cam) {
		floorplan = &gt_floorplan;
		camera = &cam;

		CountWalls(gt_floorplan, cam);

		renderer.Configure(cam);
		renderer.Render(gt_floorplan);
		// See comments in SimpleRenderer::SmoothInfiniteDepths() for why this is necessary
		int n = renderer.renderer().SmoothInfiniteDepths();
		if (n > 0) {
			DLOG << "Note: smoothing over "<<n<<" pixels with infinite depth in ground truth";
		}
	}

	double ManhattanGroundTruth::zfloor() const {
		return floorplan->zfloor();
	}

	double ManhattanGroundTruth::zceil() const {
		return floorplan->zceil();
	}

	void ManhattanGroundTruth::CountWalls(const proto::FloorPlan& fp,
																				const PosedCamera& pc) {
		// First strip out the NaN vertices
		vector<Vec2> verts;
		for (int i = 0; i < fp.vertices_size(); i++) {
			Vec2 v = asToon(fp.vertices(i));
			if (!isnan(v[0]) && !isnan(v[1])) {
				verts.push_back(v);
			}
		}

		// Compute rectifier in retina coords
		Mat3 ret_vrect = GetVerticalRectifierInRetina(pc);
		Mat3 ret_vrect_inv = LU<3>(ret_vrect).get_inverse();
		Bounds2D<> ret_vrect_bounds = Bounds2D<>::ComputeBoundingBox
			(pc.retina_bounds().GetPolygon().Transform(ret_vrect));
		Interval xbounds(ret_vrect_bounds.left(), ret_vrect_bounds.right());

		// Note that the floorplan vertices do _not_ wrap around
		Vec3 focal_line = makeVector(0, -1, 1e-6);  // cam coords: corresponds to {y>eps}

		vector<Vec3> lines;
		vector<Interval> intervals;
		for (int i = 0; i < verts.size()-1; i++) {
			Vec3 a_cam = ret_vrect * (pc.pose() * concat(verts[i], 0.0));
			Vec3 b_cam = ret_vrect * (pc.pose() * concat(verts[i+1], 0.0));
			Vec3 a_flat = makeVector(a_cam[0], a_cam[2], 1.0);  // y coord no longer matters, and now homogeneous
			Vec3 b_flat = makeVector(b_cam[0], b_cam[2], 1.0);  // y coord no longer matters, and now homogeneous
			bool nonempty = ClipAgainstLine(a_flat, b_flat, focal_line, -1);
			// this must come after clipping...
			if (nonempty) {
				Vec2 a0 = project(a_flat);
				Vec2 b0 = project(b_flat);
				Interval iv(a0[0]/a0[1], b0[0]/b0[1]);
				intervals.push_back(iv);
				lines.push_back(a_flat ^ b_flat);
			}
		}

		// Compare polygons
		MatI interval_comp(verts.size()-1, verts.size()-1);
		for (int i = 0; i < intervals.size(); i++) {
			for (int j = 0; j < intervals.size(); j++) {
				if (i==j) continue;
				Interval isct = Intersect(intervals[i], intervals[j]);
				if (isct.empty()) {
					interval_comp[i][j] = i>j ? 1 : -1;
				} else {
					double m = (isct.hi + isct.lo) / 2.0;
					double zi = -lines[i][2] / (lines[i][0]*m + lines[i][1]);
					double zj = -lines[j][2] / (lines[j][0]*m + lines[j][1]);
					interval_comp[i][j] = zi > zj ? 1 : -1;
				}
			}
		}

		// Generate tokens
		vector<Token> tokens;
		for (int i = 0; i < intervals.size(); i++) {
			tokens.push_back(Token(intervals[i].lo, i, true));
			tokens.push_back(Token(intervals[i].hi, i, false));
		}
		sort_all(tokens);

		// Initialize
		nocclusions = 0;
		nwalls = 1;  // walls are only counted if their leftmost edge is
                 // inside the image bounds but there will always be
                 // exactly one visible wall with leftmost edge
                 // outside the image

		// Walk from left to right
		int prev = -1;
		VecI active(intervals.size(), 0);
		TableComp comp(interval_comp);
		priority_queue<int,vector<int>,TableComp> heap(comp);
		for (int i = 0; i < tokens.size() && tokens[i].x < xbounds.hi; i++) {
			// Update the heap
			if (tokens[i].activate) {
				active[tokens[i].index] = true;
				heap.push(tokens[i].index);
			} else {
				active[tokens[i].index] = false;
				while (!heap.empty() && !active[heap.top()]) {  // okay for heap to be temporarily empty
					heap.pop();
				}
			}

			// Count the walls
			if (i == tokens.size()-1) {
				DLOG << "Warning: Floorplan does not fully span the image in CountWalls().\n"
						 << "Continuing for now, but the depth errors will likely be wrong.\n"
						 << EXPR_STR(tokens[i].x)
						 << EXPR_STR(xbounds.hi)
						 << EXPR_STR(tokens.size());
			} else if (tokens[i+1].x > tokens[i].x+1e-8) {
				CHECK(!heap.empty()) << "Floorplan does not fully span the image";
				if (heap.top() != prev && tokens[i].x > xbounds.lo) {
					nwalls++;
					if (prev == -1 || RingDist<int>(prev, heap.top(), intervals.size()) > 1) {
						nocclusions++;
					}
				}
				prev = heap.top();
			}
		}
	}

}
