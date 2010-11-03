#include "bld_helpers.h"
#include <math.h>

#include <boost/filesystem.hpp>
#include <LU.h>

#include "common_types.h"
#include "guided_line_detector.h"
#include "manhattan_dp.h"
#include "vars.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"
#include "clipping.h"
#include "camera.h"
#include "floorplan_renderer.h"
#include "geom_utils.h"

#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	lazyvar<string> gvSequencesDir("Sequences.DataDir");
	lazyvar<string> gvMapPath("Sequences.MapPath");

	string GetMapPath(const string& sequence_name) {
		fs::path file = fs::path(*gvSequencesDir) / sequence_name / *gvMapPath;
		CHECK_PRED1(fs::exists, file) << "No map file for sequence: " << sequence_name;
		return file.string();
	}

	void GetTrueOrients(const proto::FloorPlan& floorplan,
	                    const PosedCamera& pc,
	                    MatI& gt_orients) {
		FloorPlanRenderer re;
		re.RenderOrients(floorplan, pc, gt_orients);
	}


	// Get the ground truth orientation map for a frame by rendering the floorplan.
	void GetGroundTruth(const proto::FloorPlan& fp,
	                    const PosedCamera& pc,
	                    MatI& orients,
											int& nw,
											int& no) {
		GetTrueOrients(fp, pc, orients);
		CountVisibleWalls(fp, pc, nw, no);
	}

	void DownsampleOrients(const MatI& in, MatI& out, const Vec2I& res) {
		MatI votes[3];
		out.Resize(res[1], res[0]);
		for (int i = 0; i < 3; i++) {
			votes[i].Resize(res[1], res[0], 0);
		}

		for (int y = 0; y < in.Rows(); y++) {
			int yy = y * res[1] / in.Rows();
			for (int x = 0; x < in.Cols(); x++) {
				int xx = x * res[0] / in.Cols();
				if (in[y][x] >= 0) {
					CHECK_LT(in[y][x], 3);
					votes[ in[y][x] ][yy][xx]++;
				}
			}
		}

		int area = (in.Rows()/res[1]) * (in.Cols()/res[0]);
		for (int y = 0; y < res[1]; y++) {
			for (int x = 0; x < res[0]; x++) {
				out[y][x] = -1;
				int maxv = area/10;  // below this threshold pixels will be "unknown"
				for (int i = 0; i < 3; i++) {
					if (votes[i][y][x] > maxv) {
						maxv = votes[i][y][x];
						out[y][x] = i;
					}
				}
			}
		}
	}

	void DownsampleOrients(const MatI& in, MatI& out, int k) {
		DownsampleOrients(in, out, makeVector(in.Cols()/k, in.Rows()/k));
	}

	double ComputeAgreementPct(const MatI& a, const MatI& b) {
		return 1.0*ComputeAgreement(a, b) / (a.Rows()*a.Cols());
	}

	int ComputeAgreement(const MatI& a, const MatI& b) {
		CHECK_EQ(a.Rows(), b.Rows());
		CHECK_EQ(a.Cols(), a.Cols());
		int n = 0;
		for (int y = 0; y < a.Rows(); y++) {
			const int* arow = a[y];
			const int* brow = b[y];
			for (int x = 0; x < a.Cols(); x++) {
				if (arow[x] == brow[x]) n++;
			}
		}
		return n;
	}

	void InterchangeLabels(MatI& m, int a, int b) {
		for (int y = 0; y < m.Rows(); y++) {
			int* row = m[y];
			for (int x = 0; x < m.Cols(); x++) {
				// note that row[x] is one of -1,0,1,2
				if (row[x] == a) {
					row[x] = b;
				} else if (row[x] == b) {
					row[x] = a;
				}
			}
		}
	}

	Mat3 GetFloorCeilHomology(const PosedCamera& pc, const proto::FloorPlan& fp) {
		double zfloor = fp.zfloor();
		double zceil = fp.zceil();
		Vec3 vup = pc.pose().inverse() * makeVector(0,1,0);
		if (Sign(zceil-zfloor) == Sign(vup[2])) {
			swap(zfloor, zceil);
		}
		return GetManhattanHomology(pc, zfloor, zceil);
	}



	// Some tools for CountVisibleWalls below...
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

	// First strip out the NaN vertices
	void CountVisibleWalls(const proto::FloorPlan& fp,
												 const PosedCamera& pc,
												 int& num_walls,
												 int& num_occlusions) {
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

		// visualize
		/*Vec2I canvas_size = makeVector(500,500);
			FileCanvas fp_canvas("out/cam_fp.png", canvas_size);
			Mat3 canvas_cam = Identity;
			canvas_cam[0] = makeVector(20, 0, canvas_size[0]/2);
			canvas_cam[1] = makeVector(0, 20, canvas_size[1]/2);
			Vec3 cam_c = canvas_cam * makeVector(0, 0, 1);
			fp_canvas.DrawDot(project(cam_c), 4.0, Colors::black());
			fp_canvas.StrokeLine(project(cam_c),
			project(canvas_cam*makeVector(xbounds.lo, 1.0, 1.0)),
			Colors::white());
			fp_canvas.StrokeLine(project(cam_c),
			project(canvas_cam*makeVector(xbounds.hi, 1.0, 1.0)),
			Colors::white());
			fp_canvas.StrokeLine(makeVector(0, canvas_size[0]/2),
			makeVector(canvas_size[1], canvas_size[0]/2),
			Colors::black());*/
		vector<Vec3> lines;
		vector<Interval> intervals;
		for (int i = 0; i < verts.size()-1; i++) {
			Vec3 a_cam = ret_vrect * (pc.pose() * concat(verts[i], 0.0));
			Vec3 b_cam = ret_vrect * (pc.pose() * concat(verts[i+1], 0.0));
			Vec3 a_flat = makeVector(a_cam[0], a_cam[2], 1.0);  // y coord no longer matters, and now homogeneous
			Vec3 b_flat = makeVector(b_cam[0], b_cam[2], 1.0);  // y coord no longer matters, and now homogeneous
			bool exists = ClipAgainstLine(a_flat, b_flat, focal_line, -1);
			// this must come after clipping...
			if (exists) {
				Vec2 a0 = project(a_flat);
				Vec2 b0 = project(b_flat);
				Interval iv(a0[0]/a0[1], b0[0]/b0[1]);
				intervals.push_back(iv);
				lines.push_back(a_flat ^ b_flat);
				/*fp_canvas.StrokeLine(project(canvas_cam*a_flat),
					project(canvas_cam*b_flat),
					BrightColors::Get(i));
					Vec2 canvas_a = project(canvas_cam*makeVector(iv.lo, 1.0, 1.0));
					Vec2 canvas_b = project(canvas_cam*makeVector(iv.hi, 1.0, 1.0));
					canvas_a[0] = Clamp<double>(canvas_a[0], 0, fp_canvas.nx());
					canvas_b[0] = Clamp<double>(canvas_b[0], 0, fp_canvas.nx());
					fp_canvas.StrokeLine(canvas_a + makeVector(0,i),
					canvas_b + makeVector(0,i),
					BrightColors::Get(i));*/
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

		//TITLED("\n\nCOMP") DREPORT(interval_comp);

		// Generate tokens
		vector<Token> tokens;
		for (int i = 0; i < intervals.size(); i++) {
			tokens.push_back(Token(intervals[i].lo, i, true));
			tokens.push_back(Token(intervals[i].hi, i, false));
		}
		sort_all(tokens);
		/*TITLED("\n\nINIT TOKENS")
			for (int i = 0; i < tokens.size(); i++) {
			TITLED(i) DREPORT(tokens[i].x, tokens[i].index, tokens[i].activate);
			}
			DREPORT(xbounds);*/

		// Initialize
		num_occlusions = 0;
		num_walls = 1;  // walls are only counted once they begin inside
		// the current view but there will always be
		// exactly one wall that begins outside the image
		// (i.e. the leftmost one)

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
			CHECK_LT(i, tokens.size()-1) << "Floorplan does not fully span the image";
			if (tokens[i+1].x > tokens[i].x+1e-8) {
				CHECK(!heap.empty()) << "Floorplan does not fully span the image";
				if (heap.top() != prev && tokens[i].x > xbounds.lo) {
					num_walls++;
					if (prev == -1 || RingDist<int>(prev, heap.top(), intervals.size()) > 1) {
						num_occlusions++;
					}
				}
				prev = heap.top();
			}
		}
	}


	void DrawPayoffs(Canvas& canvas,
									 const boost::array<MatF,2>& payoffs,
									 const DPGeometry& geom) {
		double max_payoff = 0;
		for (int i = 0; i < 2; i++) {
			for (int y = 0; y < payoffs[i].Rows(); y++) {
				const float* row = payoffs[i][y];
				for (int x = 0; x < payoffs[i].Cols(); x++) {
					if (row[x] > max_payoff) max_payoff = row[x];
				}
			}
		}

		static const double kDotSize = 1.0;
		for (int i = 0; i < 2; i++) {
			Vec2 tdot = makeVector(kDotSize*i*1.5, 0);
			for (int y = 0; y < payoffs[i].Rows(); y++) {
				const float* row = payoffs[i][y];
				for (int x = 0; x < payoffs[i].Cols(); x++) {
					if (row[x] >= 0) {
						double v = row[x] / max_payoff;
						PixelRGB<byte> color( (i == 1 ? v*255 : 0),
																	(i == 0 ? v*255 : 0),
																	0);
						Vec2 p = project(geom.GridToImage(makeVector(x,y)));
						canvas.DrawDot(p+tdot, kDotSize, color);
					}
				}
			}
		}
	}

	void OutputPayoffsViz(const string& filename,
												const ImageRGB<byte>& orig,
												const boost::array<MatF,2>& payoffs,
												const DPGeometry& geom) {
		FileCanvas canvas(filename, orig);
		DrawPayoffs(canvas, payoffs, geom);
	}
}
