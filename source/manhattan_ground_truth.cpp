#include "manhattan_ground_truth.h"

#include <queue>

#include <TooN/LU.h>
#include <TooN/SVD.h>

#include "common_types.h"
#include "floorplan_renderer.h"
#include "camera.h"
#include "map.pb.h"
#include "clipping.h"
#include "geom_utils.h"
#include "manhattan_dp.h"
#include "protobuf_utils.h"
#include "bld_helpers.h"

#include "integral_image.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "numeric_utils.tpp"

namespace indoor_context {
	using namespace toon;

	// Backprojects rays through a fixed camera onto a fixed plane
	class RayCaster {
	public:
		mutable SVD<4> svd;  // it seems SVD.backsub is non-const
		Vec3 depth_eqn;

		// Initialize empty
		RayCaster();
		// Initialize and compute
		RayCaster(const Matrix<3,4>& camera, const Vec4& plane);
		// Compute
		void Compute(const Matrix<3,4>& camera, const Vec4& plane);
		// Back-project from image into plane in world
		Vec3 BackProject(const Vec2& p) const;
		// Get the depth of the plane at a point p
		double GetDepthAt(const Vec2& p) const;
	};

	RayCaster::RayCaster() {	}

	RayCaster::RayCaster(const Matrix<3,4>& camera, const Vec4& plane) {
		Compute(camera, plane);
	}

	void RayCaster::Compute(const Matrix<3,4>& camera, const Vec4& plane) {
		Mat4 M;
		M.slice<0, 0, 3, 4> () = camera;
		M.slice<3, 0, 1, 4> () = plane.as_row();
		svd.compute(M);
		double du = 1. / (camera[2] * atretina(svd.backsub(makeVector(1,0,1,0))));
		double dv = 1. / (camera[2] * atretina(svd.backsub(makeVector(0,1,1,0))));
		double dw = 1. / (camera[2] * atretina(svd.backsub(makeVector(0,0,1,0))));
		depth_eqn = makeVector(du-dw, dv-dw, dw);
	}

	Vec3 RayCaster::BackProject(const Vec2& p) const {
		return project(svd.backsub(makeVector(p[0], p[1], 1., 0.)));
	}

	double RayCaster::GetDepthAt(const Vec2& p) const {
		return p[0]*depth_eqn[0] + p[1]*depth_eqn[1] + depth_eqn[2];
	}


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
			Vec3 a_flat = makeVector(a_cam[0], a_cam[2], 1.0);  // y no longer matters, and now homog
			Vec3 b_flat = makeVector(b_cam[0], b_cam[2], 1.0);  // y no longer matters, and now homog
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
			floor_segments.push_back(LineSegment(pc.WorldToIm(concat(xs[i], fp.zfloor())),
																					 pc.WorldToIm(concat(xs[i+1], fp.zfloor()))));
			ceil_segments.push_back(LineSegment(pc.WorldToIm(concat(xs[i], fp.zceil())),
																					pc.WorldToIm(concat(xs[i+1], fp.zceil()))));
			segment_orients.push_back(renderer.GetWallOrientation(xs[i], xs[i+1]));
		}
	}

	void ManhattanGroundTruth::ComputePathAndAxes(const DPGeometry& geometry,
																								VecI& path,
																								VecI& axes) const {
		ComputePathAndOrients(geometry, path, axes);
		for (int i = 0; i < axes.Size(); i++) {
			axes[i] = 1-axes[i];
		}
	}

	void ManhattanGroundTruth::ComputePathAndOrients(const DPGeometry& geometry,
																									 VecI& path,
																									 VecI& orients) const {
		int nx = geometry.grid_size[0];
		int ny = geometry.grid_size[1];
		path.Resize(nx, -1);
		orients.Resize(nx, -1);

		// Create both floor and ceiling paths
		VecI floor_path, floor_orients;
		VecI ceil_path, ceil_orients;
		SegmentsToPathUnclamped(floor_segments, segment_orients, geometry,
														floor_path, floor_orients);
		SegmentsToPathUnclamped(ceil_segments, segment_orients, geometry,
														ceil_path, ceil_orients);
		CHECK_EQ(floor_path.Size(), nx);
		CHECK_EQ(ceil_path.Size(), nx);

		// Compare number of clips for each
		int floor_clips = 0, ceil_clips = 0;
		for (int i = 0; i < nx; i++) {
			if (floor_path[i] < 0 || floor_path[i] >= ny) {
				floor_clips++;
			}
			if (ceil_path[i] < 0 || ceil_path[i] >= ny) {
				ceil_clips++;
			}
		}

		// Choose the best
		if (floor_clips < ceil_clips) {
			path = floor_path;
			orients = floor_orients;
		} else {
			path = ceil_path;
			orients = ceil_orients;
		}

		// Clip and sanity check results
		for (int x = 0; x < nx; x++) {
			CHECK_INTERVAL(orients[x], 0, 1) << "Invalid orientation at at x="<<x;
			path[x] = Clamp<int>(path[x], 0, ny-1);
		}
	}

	void ManhattanGroundTruth::ComputePathPairUnclamped(const DPGeometry& geometry,
																											VecI& ceil_path,
																											VecI& floor_path) const {
		VecI ceil_orients, floor_orients;
		SegmentsToPathUnclamped(floor_segments, segment_orients, geometry,
														floor_path, floor_orients);
		SegmentsToPathUnclamped(ceil_segments, segment_orients, geometry,
														ceil_path, ceil_orients);
	}

	void ManhattanGroundTruth::ComputeL1LossTerms(const DPGeometry& geometry,
																								MatF& loss_terms) const {
		int nx = geometry.grid_size[0];
		int ny = geometry.grid_size[1];
		loss_terms.Resize(ny, nx, 0.);

		VecI ceil_path;
		VecI floor_path;
		ComputePathPairUnclamped(geometry, ceil_path, floor_path);
		
		for (int y = 0; y < ny; y++) {
			for (int x = 0; x < nx; x++) {
				float gt_y = (y < geometry.horizon_row) ? ceil_path[x] : floor_path[x];
				loss_terms[y][x] = abs(y - gt_y);
			}
		}
	}

	void ManhattanGroundTruth::ComputeLabellingLossTerms(const DPGeometry& geometry,
																											 MatF& loss_terms) const {
		int nx = geometry.grid_size[0];
		int ny = geometry.grid_size[1];
		loss_terms.Resize(ny, nx, 0.);
		loss_terms.Fill(0.);

		// Compute the path
		VecI gt_ceil_path;
		VecI gt_floor_path;
		ComputePathPairUnclamped(geometry, gt_ceil_path, gt_floor_path);

		// Clamp the true label locations so that we only count those
		// labels inside the image bounds
		for (int x = 0; x < nx; x++) {
			gt_ceil_path[x] = Clamp<int>(gt_ceil_path[x], 0, ny-1);
			gt_floor_path[x] = Clamp<int>(gt_floor_path[x], 0, ny-1);
		}

		// Compute pixel improtances
		MatF importances;
		geometry.ComputeGridImportances(importances);
		IntegralColImage<float> int_importance(importances);

		// Compute loss terms
		// Note: Errors are introduced into this algorithm by rounding of the
		// ceiling and floor positions to integers. It adds up to an error
		// magnitude of O(image width) since there is O(1 pixel) error in
		// each column (averaging half a pixel at floor and ceil)
		for (int y = 0; y < ny; y++) {
			for (int x = 0; x < nx; x++) {
				float opp_y = geometry.Transfer(makeVector(1.*x,y))[1];
				int ceil_y = Clamp<int>(roundi(min(y, opp_y)), 0, ny-1);
				int floor_y = Clamp<int>(roundi(max(y, opp_y)), 0, ny-1);
				// Thankfully, the order of the second two parameters doesn't
				// matter; inverting them will simply negate the return value
				float ceil_err = abs(int_importance.Sum(x, ceil_y, gt_ceil_path[x]));
				float floor_err = abs(int_importance.Sum(x, floor_y, gt_floor_path[x]));
				float err = ceil_err + floor_err;
				loss_terms[y][x] = err;
			}
		}		
	}

	void ManhattanGroundTruth::ComputeDepthLossTerms(const DPGeometryWithScale& geometry,
																									 MatF& loss_terms) const {
		int nx = geometry.grid_size[0];
		int ny = geometry.grid_size[1];
		loss_terms.Resize(ny, nx, 0.);

		// Get ground truth
		//const MatD& gt_depth = depthmap();

		// Compute pixel improtances
		MatF importances;
		geometry.ComputeGridImportances(importances);

		// Compute some geometric entities
		Matrix<3,4> cam = geometry.imageToGrid * geometry.camera->Linearize();
		Vec3 princ = unit(geometry.camera->GetPrincipalDirection());
		Vec3 normal = princ ^ GetAxis<3>(kVerticalAxis);
		Vec4 ceil_plane = makeVector(0, 0, 1, -geometry.zceil);
		Vec4 floor_plane = makeVector(0, 0, 1, -geometry.zfloor);
		Vec3 ceil_deqn = PlaneToDepthEqn(cam, ceil_plane);
		Vec3 floor_deqn = PlaneToDepthEqn(cam, floor_plane);

		// For grid loss terms
		FloorPlanRenderer rend;
		rend.Configure(cam, geometry.grid_size);
		rend.Render(*floorplan);
		rend.renderer().SmoothInfiniteDepths();
		const MatD& gt_depth = rend.depthmap();

		RayCaster floor_caster(cam, floor_plane);
		RayCaster ceil_caster(cam, ceil_plane);
		RayCaster wall_casters[ny];
		for (int y = 0; y < ny; y++) {
			const RayCaster& caster = y < geometry.horizon_row ? ceil_caster : floor_caster;
			Vec3 pt0 = caster.BackProject(makeVector<double>(0., y));
			Vec3 pt1 = caster.BackProject(makeVector<double>(nx, y));
			Vec3 wall_nrm = (pt0 - pt1) ^ GetAxis<3>(kVerticalAxis);
			Vec4 wall_plane = concat(wall_nrm, -wall_nrm * pt0);
			wall_casters[y].Compute(cam, wall_plane);
		}

		// Compute losses
		for (int y = 0; y < ny; y++) {
			const RayCaster& wall_caster = wall_casters[y];
			const Vec3& wall_deqn = wall_caster.depth_eqn;
			for (int x = 0; x < nx; x++) {
				float ceil_y, floor_y;
				geometry.GetWallExtentUnclamped(makeVector<double>(x,y), ceil_y, floor_y);

				float ceil_deqn_b = ceil_deqn[0]*x + ceil_deqn[2];
				float ceil_deqn_a = ceil_deqn[1];

				float floor_deqn_b = floor_deqn[0]*x + floor_deqn[2];
				float floor_deqn_a = floor_deqn[1];

				float wall_deqn_b = wall_deqn[0]*x + wall_deqn[2];
				float wall_deqn_a = wall_deqn[1];

				double loss_sum = 0.;
				for (int yy = 0; yy < ny; yy++) {
					float hyp;
					if (yy < ceil_y) {
						hyp = 1. / (ceil_deqn_a*yy + ceil_deqn_b);
					} else if (yy < floor_y) {
						hyp = 1. / (wall_deqn_a*yy + wall_deqn_b);
					} else {
						hyp = 1. / (floor_deqn_a*yy + floor_deqn_b);
					}
					float gt = gt_depth[yy][x];
					float w = importances[yy][x];
					loss_sum += w * abs(gt-hyp) / max(gt,hyp);
				}
				loss_terms[y][x] = loss_sum;
			}
		}
	}

	void ManhattanGroundTruth::DrawOrientations(ImageRGB<byte>& canvas, float alpha) const{
		indoor_context::DrawOrientations(orientations(), canvas, alpha);
	}

	void ManhattanGroundTruth::DrawDepthMap(ImageRGB<byte>& canvas) const {
		DrawMatrixRescaled(depthmap(), canvas);
	}

	void ManhattanGroundTruth::OutputOrientations(const ImageRGB<byte>& bg,
																								const string& file) const {
		ImageRGB<byte> canvas;
		ImageCopy(bg, canvas);
		DrawOrientations(canvas, 0.35);
		WriteImage(file, canvas);
	}

	void ManhattanGroundTruth::OutputDepthMap(const string& file) const {
		ImageRGB<byte> canvas;
		DrawDepthMap(canvas);
		WriteImage(file, canvas);
	}
}
