// Find optimal indoor Manhattan structures by dynamic programming

#include <VW/Image/imagecopy.tpp>

#include <SVD.h>
#include <LU.h>

#include <VNL/Algo/svd.h>

#include "common_types.h"
#include "guided_line_detector.h"
#include "building_estimator.h"
#include "vars.h"
#include "map.h"
#include "geom_utils.h"
#include "viewer3d.h"
#include "clipping.h"
#include "camera.h"

#include "math_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"
#include "range_utils.tpp"

using namespace indoor_context;
using namespace toon;

// Represents a mapping between floor and ceiling
class FloorCeilMapping {
public:
	Vector<3> up;  // vertical vanishing point
	double zFloor, zCeil;  // z coord of floor and ceiling in the world
	Matrix<3> FtoC, CtoF;  // homography and its inverse
	const PosedCamera* pc;

	void Compute(const Vector<3>& ret_floor_pt,
							 const Vector<3>& ret_ceil_pt,
							 const PosedCamera& cam) {
		pc = &cam;
		up = cam.GetRetinaVpt(2);

		// Solve for depth of floor and ceiling
		Matrix<3,2> m;
		m.slice<0,0,3,1>() = ret_ceil_pt.as_col();
		m.slice<0,1,3,1>() = -ret_floor_pt.as_col();
		Vector<2> soln = SVD<3,2>(m).backsub(up);
		zFloor = soln[1]*ret_floor_pt[2];
		zCeil = soln[1]*ret_floor_pt[2];

		// Construct the homography
		FtoC = (soln[1]*up*ret_floor_pt)*Identity + up.as_col()*up.as_row();
		CtoF = LU<3>(FtoC).get_inverse();
	}

	bool BelowHorizon(const Vector<3>& ret_x) {
		return Sign(ret_x*up) == Sign(up[1]);
	}

	bool AboveHorizon(const Vector<3>& ret_x) {
		return !BelowHorizon(ret_x);
	}

	bool BelowHorizonIm(const Vector<3>& im_x) {
		return BelowHorizon(pc->ImToRet(im_x));
	}

	bool AboveHorizonIm(const Vector<3>& im_x) {
		return AboveHorizon(pc->ImToRet(im_x));
	}

	Vector<3> Transfer(const Vector<3>& ret_x) {
		return (BelowHorizon(ret_x) ? FtoC : CtoF) * ret_x;
	}

	Vector<3> TransferIm(const Vector<3>& im_x) {
		return pc->RetToIm(Transfer(pc->ImToRet(im_x)));
	}
};

// Get the matrix for a rotation about an axis u by an angle t
Matrix<3> BuildRotation(const Vector<3>& axis,
												const double& sin_t,
												const double& cos_t) {
	// doesn't seem to work
	Vector<3> u = unit(axis);
	Matrix<3> skew;
	skew[0] = makeVector(0, -u[2], u[1]);
	skew[1] = makeVector(u[2], 0, -u[0]);
	skew[2] = makeVector(-u[1], u[0], 0);
	Matrix<3> outer = u.as_col()*u.as_row();
	return outer + cos_t*(Identity-outer) + sin_t*skew;
}

// Get the matrix for a rotation about an axis u by an angle t
Matrix<3> BuildRotation(const Vector<3>& u, const double& t) {
	return BuildRotation(u, sin(t), cos(t));
}



class ManhattanDP {
public:
	struct SearchNode {
		double score;
		int src_axis;
		int src_isct;
	};

	FloorCeilMapping fcmap;
	PosedCamera pc;

	double vpt_xs[3];
	const vector<LineSeg>* ret_clines;  // array!

	vector<double> div_xs;
	scoped_array<vector<pair<double, int> > > isct_ys[2];
	MatI cumul_orients[3];

	map<pair<int, pair<int, int> >, SearchNode> out_cache;
	map<pair<int, pair<int, int> >, double> in_cache;
	int access, cache_hits;

	void Compute(const vector<LineSeg> retina_clines[],
							 const PosedCamera& pcam,
							 const MatI& est_orients) {
		pc = pcam;
		ret_clines = retina_clines;

		// Allocate and copy the first row
		for (int i = 0; i < 3; i++) {
			vpt_xs[i] = project(pc.GetRetinaVpt(i))[0];
			cumul_orients[i].Resize(est_orients.Rows(), est_orients.Cols(), 0);
			copy(est_orients[0], est_orients[0]+est_orients.Cols(), cumul_orients[i][0]);
		}

		// Build the integral-col images
		for (int y = 1; y < est_orients.Rows(); y++) {
			const int* inrow = est_orients[y];
			for (int i = 0; i < 3; i++) {
				const int* prevrow = cumul_orients[i][y-1];
				int* outrow = cumul_orients[i][y];
				for (int x = 0; x < est_orients.Cols(); x++) {
					outrow[x] = inrow[x] == i ? prevrow[x]+1 : prevrow[x];
				}
			}
		}

		// Compute div columns
		div_xs.clear();
		int num_hline_divs = 0, num_culled_divs = 0;

		// Add divs for each vertical line
		BOOST_FOREACH (const LineSeg& seg, ret_clines[2]) {
			div_xs.push_back(project(seg.midpoint())[0]);
		}

		// Add divs for each intersection of horizontal lines
		BOOST_FOREACH (const LineSeg& a, ret_clines[0]) {
			BOOST_FOREACH (const LineSeg& b, ret_clines[1]) {
				double x = project(a.eqn() ^ b.eqn())[0];
				// ignore y coord in bounds test
				if (pc.camera.retina_bounds().Contains(makeVector(x,0))) {
					num_hline_divs++;
					div_xs.push_back(x);
				}
			}
		}
		sort_all(div_xs);

		// Remove divs less than k pixels apart
		double kSep = GV3::get<double>("ManhattanDP.MinDivSeperation");
		double thresh = pc.camera.GetRetinaPixelSize()[0];
		for (int i = div_xs.size()-2; i >= 0; i--) {
			if (div_xs[i+1]-div_xs[i] < thresh) {
				div_xs.erase(div_xs.begin()+i);
				num_culled_divs++;
			}
		}

		DLOG << "Num divs:";
		INDENTED {
			DLOG << "From vlines: " << ret_clines[2].size();
			DLOG << "From hline intersections: " << num_hline_divs;
			INDENTED DLOG << "(of " << ret_clines[0].size()*ret_clines[1].size() << ")";
			DLOG << "Culled: " << num_culled_divs;
			DLOG << "Final: " << div_xs.size();
		}

		// Compute the intersections
		isct_ys[0].reset(new vector<pair<double, int> >[div_xs.size()]);
		isct_ys[1].reset(new vector<pair<double, int> >[div_xs.size()]);
		for (int i = 0; i < div_xs.size(); i++) {
			Vector<3> div_eqn = makeVector(1, 0, -div_xs[i]);

			for (int j = 0; j < 2; j++) {
				COUNTED_FOREACH(int k, const LineSeg& seg, ret_clines[j]) {
					double y = project(seg.eqn() ^ div_eqn)[1];
					if (pc.camera.retina_bounds().Contains(makeVector(0, y))) {
						isct_ys[j][i].push_back(make_pair(y, k));
					}
				}
				sort_all(isct_ys[j][i], compose_twice(less<double>(),
																							&select1st<pair<double, int> >));
			}
		}

		cache_hits = 0;
		access = 0;
		double best = SolveOut(div_xs.size()-1, 0, 0);
		DREPORT(access, cache_hits, best);
	}

	int OrientColumnCount(int axis, int x, int y0, int y1) {
		CHECK_GE(y1,y0);
		return cumul_orients[axis][y1][x] - cumul_orients[axis][y0][x];
	}

	// Cache for both SolveIn and SolveOut because the cost calculation
	// in SolveIn is significant, while SolveOut contains the recursive
	// relation, so this increases speed (though not complexity)
	// considerably.
	double SolveIn(int div, int in_isct, int in_axis) {
		access++;
		// lookup cache
		pair<int, pair<int, int> > key = make_pair(div, make_pair(in_isct, in_axis));
		if (in_cache.find(key) == in_cache.end()) {
			double score = SolveIn_Impl(div, in_isct, in_axis);
			in_cache[key] = score;
			return score;
		} else {
			cache_hits++;
			return in_cache[key];
		}
	}

	// Cache for both SolveIn and SolveOut because the cost calculation
	// in SolveIn is significant, while SolveOut contains the recursive
	// relation, so this increases speed (though not complexity)
	// considerably.
	double SolveOut(int div, int out_isct, int out_axis) {
		access++;
		// lookup cache
		pair<int, pair<int, int> > key = make_pair(div, make_pair(out_isct, out_axis));
		if (out_cache.find(key) == out_cache.end()) {
			SearchNode node = SolveOut_Impl(div, out_isct, out_axis);
			out_cache[key] = node;
			return node.score;
		} else {
			cache_hits++;
			return out_cache[key].score;
		}
	}


	double SolveIn_Impl(int div, int in_isct, int in_axis) {
		CHECK_GT(div, 0);
		const vector<pair<double, int> >& prev_iscts = isct_ys[in_axis][div-1];
		for (int i = 0; i < prev_iscts.size(); i++) {
			if (prev_iscts[i].second == in_isct) {
				double score = SolveOut(div-1, i, in_axis);
				int x0 = roundi(pc.RetToIm(makeVector(div_xs[div-1], 0))[0]);
				int x1 = roundi(pc.RetToIm(makeVector(div_xs[div], 0))[0]);
				double y0 = prev_iscts[in_isct].first;  /// WAS I
				double y1 = isct_ys[in_axis][div][in_isct];

				double m = (y1-y0)/(x1-x0);
				double c = y0 - x0*m;

				for (int xx = x0; xx <= x1; xx++) {
					int yceil = roundi(c+m*xx);
					int yfloor = roundi(fcmap.TransferIm(makeVector(xx, c+m*xx)));
					score += OrientColumnCount(2, xx, 0, yceil);  // score ceiling
					score += OrientColumnCount(1-in_axis, xx, yceil, yfloor);  // score wall seg
					score += OrientColumnCount(2, xx, yfloor, cumul_orients[0].Rows());  // score floor
				}

				return score;
			}
		}
		return -INFINITY;
	}									

	SearchNode SolveOut_Impl(int div, int out_isct, int out_axis) {
		if (div == 0) {
			SearchNode base = {0, -1, -1};
			return base;
		} else {
			SearchNode best;

			// recursive case
			double div_x = div_xs[div];
			Vector<3> out_vpt = pc.GetRetinaVpt(out_axis);
			double out_vpt_x = project(out_vpt)[0];
			double out_isct_y = isct_ys[out_axis][div][out_isct].first;

			for (int in_axis = 0; in_axis < 2; in_axis++) {
				const vector<pair<double, int> >& iscts = isct_ys[in_axis][div];
				for (int i = 0; i < iscts.size(); i++) {
					bool valid;
					if (i == out_isct) {
						valid = true;  // can always pass straight through
					} else {
						Vector<3> in_vpt = pc.GetRetinaVpt(in_axis);
						double in_vpt_x = project(in_vpt)[0];
						double in_isct_y = iscts[i].first;

						// is the occluding stripe to the left or right?
						int occl_side = in_isct_y < out_isct_y ? -1 : 1;

						int occl_axis = in_isct_y < out_isct_y ? in_axis : out_axis;
						int occl_vpt_x = project(pc.GetRetinaVpt(occl_axis))[0];
						int occl_vpt_side = occl_vpt_x < div_x ? -1 : 1;

						int opp_axis = 1-occl_axis;  // irrespective of whether in_axis=out_axis!
						int opp_vpt_x = project(pc.GetRetinaVpt(opp_axis))[0];
						int opp_vpt_side = opp_vpt_x < div_x ? -1 : 1;

						// is the occluding vpt on the same side as the occluding stripe?
						int occl_vpt_behind = occl_side == occl_vpt_side;

						// is the opposite vpt between the div and the occluding vpt
						int opp_vpt_between =
							opp_vpt_side == occl_vpt_side &&
							abs(div_x - opp_vpt_x) < abs(div_x - occl_vpt_x);

						// the occlusion is valid iff occl_vpt_behind == opp_vpt_between
						valid = occl_vpt_behind == opp_vpt_between;
					}

					if (valid) {
						// recurse
						double score = SolveIn(div, i, in_axis);
						if (score > score) {
							best.score = score;
							best.src_isct = i;
							best.src_axis = in_axis;
						}
					}
				}
			}
			return best;
		}
	}
};



int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);

	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Instantiate the viewer
	Viewer3D viewer;

	// Load the map
	Map map;
	map.Load();
	map.scene_to_slam = SO3<>::exp(makeVector(0.0466705, 1.58136, -0.0029917));
	WITHOUT_DLOG map.RotateToSceneFrame();
	DREPORT(map.scene_to_slam.ln());

	// Chose a keyframe arbitrarily
	const KeyFrame& kf = map.kfs[0];

	// Prepare the image
	PosedImage pim(kf.pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	// Detect lines
	GuidedLineDetector ldet(pim);
	ldet.OutputRayViz("out/rays.png");
	ldet.OutputSegmentsViz("out/segments.png");

	// These were selected by hand...
	const int est_axis = 1;
	const int floor_i = 2;
	const int ceil_i = 12;

	// Select the midpoint
	const LineDetection& f_det = ldet.detections[est_axis][floor_i];
	const LineDetection& c_det = ldet.detections[est_axis][ceil_i];
	Vector<3> im_pfloor = f_det.seg.midpoint();
	Vector<3> im_pceil = c_det.eqn ^ (im_pfloor ^ kf.pc.GetImageVpt(2));
	Vector<3> up = kf.pc.GetRetinaVpt(2);

	ManhattanDP mdp;

	// Compute homography between floor and ceiling
	mdp.fcmap.Compute(kf.pc.ImToRet(im_pfloor),
										kf.pc.ImToRet(im_pceil),
										kf.pc);

	// Build a rotation to move the vertical vanishing point to infinity.
	// R must satisfy:
	//   (1) R*up = [0,1,0]
	//   (2) R is as close to the identity as possible
	//        (so that the other vpts are minimally affected)
	Matrix<3> R_up;
	R_up[0] = up ^ GetAxis<3>(2);
	R_up[1] = up;
	R_up[2] = R_up[0] ^ R_up[1];

	// Vanishing points
	Vector<3> ret_vpts[] = { kf.pc.GetRetinaVpt(0),
													 kf.pc.GetRetinaVpt(1),
													 kf.pc.GetRetinaVpt(2) };

	// Transfer and rotate the lines
	vector<LineSeg> ret_clines[3];
	for (int i = 0; i < 3; i++) {
		COUNTED_FOREACH(int j, const LineDetection& det, ldet.detections[i]) {
			Vector<3> a = kf.pc.ImToRet(det.seg.start);
			Vector<3> b = kf.pc.ImToRet(det.seg.end);

			// Check that this line segment doesn't cross any vpt epipoles
			bool valid = true;
			for (int k = 0; k < 3; k++) {
				if (k != i && PointSign(a,ret_vpts[k]) != PointSign(b,ret_vpts[k])) {
					valid = false;
					break;
				}
			}

			if (valid) {
				// Transfer horizontal segments from floor to ceiling
				if (i != 2 && mdp.fcmap.BelowHorizon(a)) {
					a = mdp.fcmap.Transfer(a);
					b = mdp.fcmap.Transfer(b);
				}
				
				// Rotate so that up is at infinity
				ret_clines[i].push_back(LineSeg(R_up*a, R_up*b));
			}
		}
	}

	// Compute line sweeps
	LineSweepInput sweep_input(kf.image);
	for (int i = 0; i < 3; i++) {
		sweep_input.image_vpts[i] = kf.pc.RetToIm(R_up*kf.pc.GetRetinaVpt(i));
		BOOST_FOREACH(const LineSeg& seg, ret_clines[i]) {
			sweep_input.lines.push_back(LineDetection(seg.start, seg.end, i));
		}
	}
	IsctGeomLabeller geom_labeller(sweep_input);

	// Begin the DP
	mdp.Compute(ret_clines, kf.pc, geom_labeller.orient_map);

	// Draw transferred lines
	ImageRGB<byte> canvas;
	ImageRGB<byte> line_canvas(kf.image.sz());
	ImageRGB<byte> cline_canvas(kf.image.sz());
	ImageCopy(kf.image.rgb, canvas);
	line_canvas.Clear(Colors::white());
	cline_canvas.Clear(Colors::white());
	for (int i = 0; i < 3; i++) {
		COUNTED_FOREACH(int j, const LineDetection& det, ldet.detections[i]) {
			const Vector<3>& a = det.seg.start;
			const Vector<3>& b = det.seg.end;

			Vector<3> tr_a = mdp.fcmap.TransferIm(a);
			Vector<3> tr_b = mdp.fcmap.TransferIm(b);
			PixelRGB<byte> color = BrightColors::Get(j);
			det.DrawLine(canvas, color, 2);
			DrawLineClipped(canvas, project(tr_a), project(tr_b), color);

			Vector<3> ca = kf.pc.RetToIm(R_up * kf.pc.ImToRet(a));
			Vector<3> cb = kf.pc.RetToIm(R_up * kf.pc.ImToRet(b));
			DrawLineClipped(line_canvas, project(a), project(b), color);
			DrawLineClipped(cline_canvas, project(ca), project(cb), color);
		}
	}
	WriteImage("out/transferred.png", canvas);
	WriteImage("out/lines.png", line_canvas);
	WriteImage("out/clines.png", cline_canvas);

	//viewer.Run();
	return 0;
}
