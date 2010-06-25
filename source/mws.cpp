#include <signal.h>
#include <google/stacktrace.h>
#include <boost/filesystem.hpp>

#include <Cholesky.h>
#include <GCoptimization.h>

#include <VW/Image/imagecopy.tpp>

#include "common_types_entry.h"
#include "vars.h"
#include "map.pb.h"
#include "mean_shift.h"
#include "timer.h"
#include "safe_stream.h"
#include "canny.h"

#include "viewer3d.h"
#include "map_widgets.h"

#include "colored_points.h"
#include "read_ply.h"

#include "math_utils.tpp"
#include "io_utils.tpp"
#include "gl_utils.tpp"
#include "image_utils.tpp"

using namespace indoor_context;
using namespace toon;

const double kFoldThresh = 2.0;
const double kFoldAttenuation = 0.01;

const double kDepthMargin = 0.1;
const double kConfThresh = 0.7;
const int kSmoothCost = 100;
const int kNumAlphaExpansions = 3;

const int kSampleStride = 50; // sample rate for mean-shift input
const double kMeanShiftWindow = 0.1; // mean shift kernel bandwidth

const int kPlaneSupportThresh = 20;  // minimum supporting points to include a point in the optimization

const int kCostMult = 10000;  // used to turn floats into ints for GCoptimization calls

struct MvsPoint {
	Vec3 world_pos;
	Vec3 world_nrm;
	vector<int> obs_frames;
	double confidence;
};

// maps: [-1  0] -> 0
//       [ 0 -1] -> 1
//       [ 0  1] -> 2
//       [ 1  0] -> 3
inline int CompassIndex(int dx, int dy) {
	return (dx + 3 * dy + 3) / 2;
}

// Compute pairwise terms for the graph cuts
class MvsSimpleSmoothCost : public GCoptimization::SmoothCostFunctor {
public:
	ImageRef dims;
	int num_labels;
	const vector<Vec3>* depth_eqns;
	const MatF* fold_map;
	void Configure(ImageRef sz, const vector<Vec3>* d_eqns, const MatF* f_map) {
		dims = sz;
		num_labels = d_eqns->size();
		depth_eqns = d_eqns;
		fold_map = f_map;
	}
	int compute(int node1, int node2, int label1, int label2) {
		if (label1 == label2) return 0;

		ImageRef p1(node1%dims.x, node1/dims.x);
		ImageRef p2(node2%dims.x, node2/dims.x);
		Vec3 midp = unproject(asToon((p1+p2)/2.0));
		double d1 = 1.0 / ((*depth_eqns)[label1]*midp);
		double d2 = 1.0 / ((*depth_eqns)[label2]*midp);

		double fold_factor = 1.0;
		if ((*fold_map)[p1.y][p1.x] > kFoldThresh ||
				(*fold_map)[p2.y][p2.x] > kFoldThresh) {
			fold_factor = kFoldAttenuation;
		}
		// TODO: restore this
		// double cost = min(10.0, fold_factor*abs(d1-d2));
		// return cost * kCostMult;
		return fold_factor * kCostMult;
	}
};

// Intersect a ray from pixel
Vec4 IntersectRay(const Vec3& pixel, const Matrix<3, 4>& camera, const Vec4& plane) {
	// Solve for x:
	//   x*w = 0  { w is the plane normal }
	//   C*x = y  { C is the camera matrix, y is its image position }
	//
	// x is the solution to Mx=b with:
	//
	// M = [ C11 C12 C13 C14 ]
	//     [ C21 C22 C23 C24 ]
	//     [ C31 C32 C33 C34 ]
	//     [ w1  w2  w3  w4  ]
	//
	// b = [ y1 y2 y3 0 ]

	Mat4 M;
	M.slice<0, 0, 3, 4> () = camera;
	M.slice<3, 0, 1, 4> () = plane.as_row();
	Vec4 b = Zeros;
	b.slice<0, 3> () = pixel;
	return SVD<4>(M).backsub(b);
}

Mat3 GetAxisPerm(int axis, double plane_offset) {
	int other1 = (axis + 1) % 3;
	int other2 = (axis + 2) % 3;
	Mat3 perm = Zeros;
	perm[0][other1] = 1;
	perm[1][other2] = 1;
	perm[2][axis] = plane_offset;
	return perm;
}

pair<Vec3, Vec3> ComputeBounds(const vector<MvsPoint>& xs) {
	Vec3 a = INFINITY * Ones;
	Vec3 b = -INFINITY * Ones;
	BOOST_FOREACH(const MvsPoint& x, xs)
				{
					for (int i = 0; i < 3; i++) {
						a[i] = min(a[i], x.world_pos[i]);
						b[i] = max(b[i], x.world_pos[i]);
					}
				}
	return make_pair(a, b);
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 4) {
		cerr << "Usage: " << argv[0]
		        << " <truthed_map.pro> <basedir> FRAMEINDEX\n";
		return -1;
	}

	const char* tru_path = argv[1];
	const char* base_path = argv[2];
	int target_index = atoi(argv[3]);

	// Load the structure and motion data
	fs::path base_dir(base_path);

	// Load the motion data
	format frame_fmt("visualize/%08d.jpg");
	format proj_fmt("txt/%08d.txt");
	ptr_vector<ImageBundle> frames;
	vector<Matrix<3, 4> > cameras;
	fs::path proj_file;
	for (int i = 0; fs::exists(proj_file = base_dir / str(proj_fmt % i)); i++) {

		// Read the projection matrix
		sifstream proj_input(proj_file.string().c_str());
		string line = consume<string> (proj_input);
		CHECK_EQ(line, "CONTOUR") << "Unknown file format: " << proj_file;
		cameras.push_back(consume<Matrix<3, 4> > (proj_input));

		// Read the image
		fs::path frame_file = base_dir / str(format("visualize/%08d.jpg") % i);
		CHECK_PRED(fs::exists, frame_file) << "Pose file exists but not "
							        << frame_file;
		frames.push_back(new ImageBundle(frame_file.string()));
	}
	DLOG	<< "Loaded " << cameras.size() << " cameras";

	// Load the structure data
	fs::path pts_path = base_dir / "models/option.txt.patch";
	sifstream pts_input(pts_path.string().c_str());

	string token = consume<string> (pts_input);
	CHECK_EQ(token, "PATCHES") << "wrong file format";

	int num_points = consume<int> (pts_input);
	vector<MvsPoint> mvs_points;
	mvs_points.reserve(num_points);
	for (int i = 0; i < num_points; i++) {
		MvsPoint pt;
		pts_input >> token;
		CHECK_EQ(token, "PATCHS");
		pt.world_pos = consume<Vec4> (pts_input).slice<0, 3> (); // homogeneous coords
		pt.world_nrm = consume<Vec4> (pts_input).slice<0, 3> (); // normal, homogeneous coords
		pt.confidence = consume<double> (pts_input); // confidence
		consume<Vec2> (pts_input); // discard two values
		int num_obs = consume<int> (pts_input);
		for (int j = 0; j < num_obs; j++) {
			pt.obs_frames.push_back(consume<int> (pts_input));
		}
		sort_all(pt.obs_frames);
		int num_aux = consume<int> (pts_input);
		for (int j = 0; j < num_aux; j++) {
			consume<int> (pts_input);
		}
		mvs_points.push_back(pt);
	}

	// Load the PLY data
	ColoredPoints ply_pts;
	fs::path ply_path = base_dir / "models/option.txt.ply";
	ReadPly(ply_path.string(), ply_pts.vs);
	// Lighten the colors for display purposes
	for (int i = 0; i < ply_pts.vs.size(); i++) {
		PixelRGB<byte>& px = ply_pts.vs[i].second;
		px.Set(127 + px.r / 2, 127 + px.g / 2, 127 + px.b / 2);
	}

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream tru_in(tru_path, ios::binary);
	CHECK(tru_map.ParseFromIstream(&tru_in)) << "Failed to read from "
						        << argv[1];

	// Transform to the Manhattan frame
	SO3<> scene_from_slam = SO3<>::exp(asToon(tru_map.ln_scene_from_slam()));
	BOOST_FOREACH(MvsPoint& pt, mvs_points) {
		pt.world_pos = scene_from_slam * pt.world_pos;
		pt.world_nrm = scene_from_slam * pt.world_nrm;
	}
	for (int i = 0; i < ply_pts.vs.size(); i++) {
		ply_pts.vs[i].first = scene_from_slam * ply_pts.vs[i].first;
	}
	Mat4 scene_from_slam_inv = Identity;
	scene_from_slam_inv.slice<0, 0, 3, 3> ()
	        = scene_from_slam.inverse().get_matrix();
	for (int i = 0; i < cameras.size(); i++) {
		cameras[i] = cameras[i] * scene_from_slam_inv;
	}

	// Compute camera centres
	vector<Vec3> camera_centres;
	for (int i = 0; i < cameras.size(); i++) {
		camera_centres.push_back(GetCameraCentre(cameras[i]));
	}

	// Pull out the target view
	CHECK_INDEX(target_index, frames);
	const ImageBundle& target_frame = frames[target_index];
	const Matrix<3, 4>& target_camera = cameras[target_index];
	const Vec3& target_cam_ctr = camera_centres[target_index];
	const ImageRef sz = target_frame.sz(); // assume all frames have the same size

	Mat4 target_camera_full = Identity;
	target_camera_full.slice<0, 0, 3, 4> () = target_camera;

	WriteImage("out/orig.png", target_frame.rgb);

	// Compute bounding box
	pair<Vec3, Vec3> bbox = ComputeBounds(mvs_points);

	// Compute folding probabilities
	Gradients target_grads;
	target_grads.Compute(target_frame);
	MatF fold_map(sz.y, sz.x);
	fold_map.Fill(0.0);
	MatF sep_fold_maps[3];
	TIMED("Compute fold probability maps")
	for (int i = 0; i < 3; i++) {
		sep_fold_maps[i].Resize(sz.y, sz.x);
		Vec2 vpt = project(target_camera * GetAxis<4> (i));
		for (int y = 0; y < sz.y; y++) {
			const PixelF* dxrow = target_grads.diffx[y];
			const PixelF* dyrow = target_grads.diffy[y]; // careful with x's and y's here!
			float* foldrow = fold_map[y];
			float* seprow = sep_fold_maps[i][y];
			for (int x = 0; x < sz.x; x++) {
				Vec2 vdir = unit(vpt - makeVector(x, y)); // direction to vanishing point
				Vec2 tangent = makeVector(dyrow[x].y, -dxrow[x].y); // tangent is perp to gradient
				seprow[x] = norm(tangent) * abs(vdir * unit(tangent));
				foldrow[x] = max(foldrow[x], seprow[x]);
			}
		}

		ImageRGB<byte> canvas(sz);
		double m = sep_fold_maps[i].AbsoluteValueMax();
		for (int y = 0; y < sz.y; y++) {
			for (int x = 0; x < sz.x; x++) {
				byte v = 255 * sep_fold_maps[i][y][x] / m;
				canvas[y][x].Set(v, v, v);
			}
		}
		DrawLineClipped(canvas, asToon(sz) / 2, vpt, Colors::red(), 1.0);
		WriteImage(str(format("out/folds_%d.png") % i), canvas);
		WriteMatrixImageRescaled("out/folds.png", fold_map);
	}

	// Generate candidate planes
	vector<Vec4> cand_planes;
	ptr_vector<QuadWidget> plane_widgets;
	for (int i = 0; i < 3; i++) {
		MeanShift1D meanshift;
		for (int j = 0; j < mvs_points.size(); j += kSampleStride) {
			meanshift.xs.push_back(mvs_points[j].world_pos[i]);
		}
		meanshift.Compute(kMeanShiftWindow);

		// Create quad widgets
		Bounds2D<> slice(bbox.first[(i+1)%3], bbox.second[(i+1)%3],
						 bbox.first[(i+2)%3], bbox.second[(i+2)%3]);
		COUNTED_FOREACH(int j, double a, meanshift.modes) {
			cand_planes.push_back(makeVector(0.0, 0, 0, a));
			cand_planes.back()[i] = -1.0;
			Matrix<3> perm = GetAxisPerm(i, a);
			QuadWidget* qw = new QuadWidget(
					perm.T() * slice.htl(),
					perm.T() * slice.htr(),
					perm.T() * slice.hbr(),
					perm.T() * slice.hbl());
			qw->color = BrightColors::Get(j);
			plane_widgets.push_back(qw);
		}
	}
	DREPORT(cand_planes.size());

	// Project the MVS points into the keyframes
	MatI pt_projs[frames.size()];
	TIMED("Project MVS points")
	for (int i = 0; i < frames.size(); i++) {
		pt_projs[i].Resize(sz.y, sz.x, 0);
		const ImageBundle& frame = frames[i];
		for (int j = 0; j < mvs_points.size(); j++) {
			Vec3 camera_pos = cameras[i] * unproject(mvs_points[j].world_pos);
			ImageRef p = asIR(project(camera_pos));
			if (camera_pos[2] > 0 && frame.contains(p)) {
				// Multiple points should never project to the same pixel
				// since we are using the MVS visibility info. In the unlikely
				// case that they do we simply overwrite the older one with
				// the newer one.
				if (binary_search_all(mvs_points[j].obs_frames, i)) {
					pt_projs[i][p.y][p.x] = j + 1;
				} else if (pt_projs[i][p.y][p.x] == 0) {
					pt_projs[i][p.y][p.x] = -j - 1;
				}
			}
		}
	}

	// Compute the reciprocal-of-depth equations
	vector<Vec3> depth_eqns[frames.size()];
	for (int i = 0; i < frames.size(); i++) {
		BOOST_FOREACH(const Vec4& plane_eqn, cand_planes) {
			depth_eqns[i].push_back(PlaneToDepthEqn(target_camera, plane_eqn));
		}
	}

	// Reconstruct each pixel
	/*ImageRGB<byte> canvas(sz);
	ImageRGB<byte> canvas2(sz);
	double maxerr = 0.0;
	for (int y = 0; y < sz.y; y++) {
		for (int x = 0; x < sz.x; x++) {
			Vec4 b = makeVector(1.0*x, 1.0*y, 1.0, 0.0); // last elem is zero so that w*x=0
			//DREPORT(M_ray, b, decomp.backsub(b), project(decomp.backsub(b)))
			Vec4 world_pt = atretina(decomp.backsub(b));
			//DREPORT(world_pt);
			double slow_depth = (target_camera * world_pt)[2];
			double fast_depth = 1.0 / (depth_eqn * makeVector(1.0*x, 1.0*y, 1));
			canvas[y][x].Set(127+Clamp(slow_depth, -127, 128));
			canvas2[y][x].Set(127+Clamp(fast_depth, -127, 128));
			maxerr = max(maxerr, abs((slow_depth-fast_depth)/slow_depth));
		}
	}
	DREPORT(maxerr);
	WriteImage(str(format("out/true_depth_%02d.png")%i), canvas);
	WriteImage(str(format("out/fast_depth_%02d.png")%i), canvas);*/




	vector<LineWidget*> agreement_lines;
	vector<int> num_agreements(cand_planes.size());
	fill_all(num_agreements, 0);

	// Setup the data costs
	int num_pixels = sz.x * sz.y;
	int num_labels = cand_planes.size();
	scoped_array<int> data_cost(new int[num_pixels * num_labels]);
	int label = 0;
	TIMED("Prepare data costs")
	for (int i = 0; i < cand_planes.size(); i++) {
		Vec4 plane_eqn = cand_planes[i];
		Vec3 depth_eqn = depth_eqns[target_index][i];  // must take reciprocal of this

		ImageRGB<byte> cost_canvas(target_frame.nx(), target_frame.ny());
		cost_canvas.Clear(Colors::black());

		for (int y = 0; y < sz.y; y++) {
			const int* proj_row = pt_projs[target_index][y];
			for (int x = 0; x < sz.x; x++) {
				double cost = 0.0;
				if (proj_row[x] != 0) {
					int pt_index = abs(proj_row[x]) - 1;
					const MvsPoint& mvs_pt = mvs_points[pt_index];
					bool visible = proj_row[x] > 0;

					// compute the reconstructed point for this pixel
					// TODO: delete b, hyp_pt_old
					/*Vec4 b = makeVector(1.0 * x, 1.0 * y, 1.0, 0.0); // last elem is zero so that w*x=0
					Vec3 hyp_pt_old = project(decomp.backsub(b));
					Vec3 hyp_pt = reproj_pixels[i][y * sz.x + x];
					CHECK_EQ(hyp_pt, hyp_pt_old);*/

					// Compute depth difference w.r.t. the viewing ray
					/*Vec3 ray = unit(target_cam_ctr - mvs_pt.world_pos);
					double d = (hyp_pt - mvs_pt.world_pos) * ray;*/

					double mvs_depth = target_camera[2]*unproject(mvs_pt.world_pos);
					double plane_depth = 1.0 / (depth_eqn*makeVector(1.0*x, 1.0*y, 1.0));
					double d = plane_depth - mvs_depth;   // do _not_ take abs here!

					//if (x==y) INDENTED DREPORT(pt.world_pos,target_cam_ctr,d, kDepthMargin);
					if (d > kDepthMargin || (visible && d < -kDepthMargin)) {
						cost = max(mvs_pt.confidence - kConfThresh, 0.0);
						if (visible) cost_canvas[y][x] = PixelRGB<byte> (127+cost*127 / kConfThresh, 127, 127);
					} else if (visible) {
						num_agreements[i]++;
						cost_canvas[y][x] = Colors::green();
						/*LineWidget* w = new LineWidget(
								mvs_pt.world_pos, hyp_pt, 1.0,
								BrightColors::Get(i));
						w->SetSelectable(false);
						agreement_lines.push_back(w);*/
					}
				}

				// TODO: check for agreement with points in auxiliary frames

				int index = (y*sz.x+x) * num_labels + i;
				data_cost[index] = cost * kCostMult; // cast to integer
			}
		}

		for (int y = 0; y < 10; y++) {
			for (int x = 0; x < 10; x++) {
				cost_canvas[y][x] = BrightColors::Get(i);
			}
		}
		WriteImage(str(format("out/costs_%d_agree%d.png") % i
				% num_agreements[i]), cost_canvas);
	}
	DREPORT(iowrap(num_agreements));

	// Compute pairwise costs
	MvsSimpleSmoothCost edge_costs;
	edge_costs.Configure(sz, &depth_eqns[target_index], &fold_map);

	// Render a sample image
	const int kLabel1 = 11;
	const int kLabel2 = 34;
	ImageRGB<byte> edge_canvas;
	ImageRGB<byte> raw_edge_canvas(sz);
	ImageCopy(target_frame.rgb, edge_canvas);
	for (int y = 0; y < sz.y; y++) {
		for (int x = 0; x < sz.x; x++) {
			int cost = edge_costs.compute(y*sz.x+x, y*sz.x+x, kLabel1, kLabel2);
			byte v = Clamp(cost * 256 / kCostMult, 0, 256);
			raw_edge_canvas[y][x].Set(v);
			PixelRGB<byte>& p = edge_canvas[y][x];
			p.Set(p.r/2, p.g/2, p.b/2 + Clamp(v, 0, 127));

		}
	}
	WriteImage("out/edge_costs.png", edge_canvas);
	WriteImage("out/raw_edge_costs.png", raw_edge_canvas);

	// Run the graph cuts
	scoped_ptr<GCoptimizationGridGraph> gc;
	try {
		gc.reset(new GCoptimizationGridGraph(sz.x, sz.y, num_labels));
		gc->setDataCost(data_cost.get());
		gc->setSmoothCostFunctor(&edge_costs);
		/*for (int i = 0; i < num_labels; i++) {
			for (int j = 0; j < num_labels; j++) {
				gc->setSmoothCost(i, j, (i == j ? 0 : kSmoothCost));
			}
		}*/

		DLOG	<< "Before optimization energy is " << gc->compute_energy();
		TIMED("Alpha expansion") gc->expansion(kNumAlphaExpansions);
		DLOG	<< "After optimization energy is " << gc->compute_energy();
	} catch (GCException e) {
		e.Report();
		exit(-1);
	}

	// Draw the result
	ImageRGB<byte> gc_canvas;
	ImageCopy(target_frame.rgb, gc_canvas);

	// Report the results
	ColoredPoints opt_pts;
	for (int y = 0; y < sz.y; y++) {
		int rowi = y * sz.x;
		PixelRGB<byte>* outrow = gc_canvas[y];
		for (int x = 0; x < sz.x; x++) {
			BlendWith(outrow[x], BrightColors::Get(gc->whatLabel(rowi + x)));
			if (x % 5 == 0 && y % 5 == 0) {
				int label = gc->whatLabel(rowi + x);
				Mat4 M_ray;
				M_ray.slice<0, 0, 3, 4> () = target_camera;
				M_ray.slice<3, 0, 1, 4> () = cand_planes[label].as_row();
				toon::SVD<4> decomp(M_ray);
				Vec4 b = makeVector(1.0 * x, 1.0 * y, 1.0, 0.0); // last elem is zero so that w*x=0
				Vec3 p = project(decomp.backsub(b));
				//PixelRGB<byte> color = BrightColors::Get(label);
				PixelRGB<byte> color = target_frame.rgb[y][x];
				opt_pts.Add(p, color);
			}
		}
	}

	WriteImage("out/plane_labels.png", gc_canvas);

	// Set up the viewer
	Viewer3D viewer("Agreements");
	// Add the .PLY points
	viewer.Add(ply_pts, 't');

	// Add the agreement lines
	ColoredPoints line_endpts;
	line_endpts.pointSize = 5.0;
	BOOST_FOREACH(LineWidget* lw, agreement_lines) {
		viewer.AddOwned(lw, 'a');
		line_endpts.Add(lw->lineseg.end, lw->color);
	}
	viewer.Add(line_endpts);

	// Add the candidate planes
	COUNTED_FOREACH(int i, QuadWidget& qw, plane_widgets) {
		qw.color = BrightColors::Get(i);
		viewer.Add(qw, 'c');
		qw.SetVisible(false);
	}

	// Schedule creation for when the GLUT loop starts
	viewer.Create();

	// Create a seperate window for the reconstructed points
	Viewer3D recon_viewer("Reconstruction");
	// Add the reconstructed points
	recon_viewer.Add(opt_pts, 'p');
	// Add the ply points
	recon_viewer.Add(ply_pts, 't');
	recon_viewer.window().SetPosition(ImageRef(700, 0));
	recon_viewer.Run();

	//GlutWindow::Loop();

	return 0;
}
