#include "manhattan_ground_truth.h"

#include <queue>

#include <TooN/LU.h>

#include "common_types.h"
#include "floorplan_renderer.h"
#include "camera.h"
#include "map.pb.h"
#include "clipping.h"
#include "geom_utils.h"
#include "manhattan_dp.h"

#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "numeric_utils.tpp"

namespace indoor_context {
	using namespace toon;

	// Some tools for ProcessWalls below
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

		ProcessWalls(gt_floorplan, cam);

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

	void ManhattanGroundTruth::ProcessWalls(const proto::FloorPlan& fp,
																					const PosedCamera& pc) {
		// Initialize
		nocclusions = 0;
		nwalls = 0;

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
		Vec3 focal_line = makeVector(0, -1, 1e-6);

		vector<Vec3> lines;
		vector<Interval> intervals;
		for (int i = 0; i < verts.size()-1; i++) {
			Vec3 a_cam = ret_vrect * (pc.pose() * concat(verts[i], 0.0));
			Vec3 b_cam = ret_vrect * (pc.pose() * concat(verts[i+1], 0.0));
			Vec3 a_flat = makeVector(a_cam[0], a_cam[2], 1.0);  // y coord no longer matters, and now homogeneous
			Vec3 b_flat = makeVector(b_cam[0], b_cam[2], 1.0);  // y coord no longer matters, and now homogeneous
			bool nonempty = ClipAgainstLine(a_flat, b_flat, focal_line, -1);
			if (nonempty) {
				// project() must come after clipping
				Vec2 a0 = project(a_flat);
				Vec2 b0 = project(b_flat);
				intervals.push_back(Interval(a0[0]/a0[1], b0[0]/b0[1]));
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
					double m = (isct.hi + isct.lo) / 2.;
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
		// Add tokens for left and right image bounds
		tokens.push_back(Token(xbounds.lo, -1, true));
		tokens.push_back(Token(xbounds.hi, -1, false));
		sort_all(tokens);

		// Walk from left to right
		vector<Vec2> xs;
		int prev = -1;
		bool inbounds = false;
		bool done = false;
		VecI active(intervals.size(), 0);
		TableComp comp(interval_comp);
		priority_queue<int,vector<int>,TableComp> heap(comp);
		for (int i = 0; !done; i++) {
			// Process next token
			bool bound_token = tokens[i].index == -1;
			if (bound_token && tokens[i].activate) {
				inbounds = true;
			} else if (bound_token && !tokens[i].activate) {
				done = true;
			} else if (tokens[i].activate) {
				active[tokens[i].index] = true;
				heap.push(tokens[i].index);
			} else {
				active[tokens[i].index] = false;
				while (!heap.empty() && !active[heap.top()]) {  // okay for heap to be temporarily empty
					heap.pop();
				}
			}

			double xcur = tokens[i].x;
			if ((tokens[i+1].x-xcur) > 1e-8) {
				int cur = heap.empty() ? -1 : heap.top();

				// Count walls
				if (inbounds && (cur != prev || bound_token)) {
					if (!done) {
						nwalls++;
						if (!bound_token && RingDist<int>(prev, cur, intervals.size()) > 1) {
							nocclusions++;
						}
					}

					// Reconstruct vertices
					vector<int> to_add;
					if (!xs.empty()) {
						to_add.push_back(prev);
					}
					if (!done) {
						to_add.push_back(cur);
					}

					// Compute vertices
					BOOST_FOREACH(int j, to_add) {
						const Vec3& line = lines[j];
						const Interval& intv = intervals[j];
						Vec2 pflat = project(line ^ makeVector(-1, xcur, 0));
						Vec3 pcam = makeVector(pflat[0], 0, pflat[1]);
						Vec3 pworld = pc.pose_inverse() * (ret_vrect_inv * pcam);
						xs.push_back(pworld.slice<0,2>());
					}
				}
				prev = cur;
			}
		}
		CHECK(xs.size()%2 == 0) << EXPR(xs.size());

		// Compile segments
		segment_orients.clear();
		floor_segments.clear();
		ceil_segments.clear();
		for (int i = 0; i < xs.size(); i += 2) {
			floor_segments.push_back(LineSeg(pc.WorldToIm(concat(xs[i], fp.zfloor())),
																			 pc.WorldToIm(concat(xs[i+1], fp.zfloor()))));
			ceil_segments.push_back(LineSeg(pc.WorldToIm(concat(xs[i], fp.zceil())),
																			pc.WorldToIm(concat(xs[i+1], fp.zceil()))));
			segment_orients.push_back(renderer.GetWallOrientation(xs[i], xs[i+1]));
		}
	}

	void ManhattanGroundTruth::ComputePath(const DPGeometry& geometry, VecI& path, VecI& orients) {
		int nx = geometry.grid_size[0];
		int ny = geometry.grid_size[1];
		path.Resize(nx, -1);
		orients.Resize(nx, -1);

		// Choose floor or ceiling segments
		const vector<LineSeg>& segs =
			(geometry.horizon_row < .5*geometry.grid_size[1]) ?
			floor_segments :
			ceil_segments;

		// Compute the path
		for (int i = 0; i < segs.size(); i++) {
			Vec3 grid_eqn = geometry.gridToImage.T() * segs[i].eqn();  // for lines we apply H^-T (inverse of transpose)
			float x0 = geometry.ImageToGrid(segs[i].start)[0];
			float x1 = geometry.ImageToGrid(segs[i].end)[0];
			if (x0 > x1) {
				swap(x0, x1);
			}
			for (int x = max(0,floori(x0)); x <= min(nx-1,ceili(x1)); x++) {
				Vec3 isct = grid_eqn ^ makeVector(1, 0, -x);
				int y = roundi(project(isct)[1]);
				// This get messy near the edge of line segments that don't meet perfectly
				path[x] = Clamp<int>(y, 0, ny-1);
				orients[x] = segment_orients[i];
			}
		}
		for (int x = 0; x < nx; x++) {
			CHECK(path[x] != -1) << "Path does not fully span image at x="<<x;
			CHECK_INTERVAL(orients[x], 0, 1) << "Invalid orientation at x="<<x;
		}
	}

	void ManhattanGroundTruth::DrawOrientations(ImageRGB<byte>& canvas, float alpha) {
		indoor_context::DrawOrientations(orientations(), canvas, alpha);
	}

	void ManhattanGroundTruth::DrawDepthMap(ImageRGB<byte>& canvas) {
		DrawMatrixRescaled(depthmap(), canvas);
	}

	void ManhattanGroundTruth::OutputOrientations(const ImageRGB<byte>& bg,
																								const string& file) {
		ImageRGB<byte> canvas;
		ImageCopy(bg, canvas);
		DrawOrientations(canvas, 0.35);
		WriteImage(file, canvas);
	}

	void ManhattanGroundTruth::OutputDepthMap(const string& file) {
		ImageRGB<byte> canvas;
		DrawDepthMap(canvas);
		WriteImage(file, canvas);
	}
}
