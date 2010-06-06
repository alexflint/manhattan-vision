#include <iomanip>
#include <fstream>
#include <iterator>
#include <set>

#include <SVD.h>

#include "clipping.h"
#include "common_types.h"
#include "map.h"
#include "vars.h"
#include "line_sweeper.h"
#include "timer.h"
#include "geom_utils.h"
#include "worker.h"
#include "clipping.h"
#include "viewer3d.h"
#include "guided_line_detector.h"
#include "map_widgets.h"

#include "line_detector.h"
#include "vanishing_points.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

lazyvar<int> gvMaxCorners("LeeRecovery.MaxCorners");
lazyvar<double> gvOcclThresh("LeeRecovery.CnrOcclusionThresh");
lazyvar<double> gvMinCornerMargin("LeeRecovery.MinCornerMargin");
lazyvar<int> gvOrientRes("LeeRecovery.OrientRes");

void ClipToPositive(Vector<3>& a, Vector<3>& b) {
	const double kClipDist = 0.1;
	if (b[2] < 0 && a[2] > 0) {
		double t = (a[2] - kClipDist) / (a[2] - b[2]);
		b = t*b + (1-t)*a;
	} else if (a[2] < 0 && b[2] > 0) {
		double t = (b[2] - kClipDist) / (b[2] - a[2]);
		a = t*a + (1-t)*b;
	}
}

void DownsampleOrients(const MatI& in, MatI& out, int res) {
	MatI votes[3];

	int area = (in.Rows()/res) * (in.Cols()/res);

	out.Resize(res, res);
	for (int i = 0; i < 3; i++) {
		votes[i].Resize(res, res, 0);
	}

	for (int y = 0; y < in.Rows(); y++) {
		int yy = y * res / in.Rows();
		for (int x = 0; x < in.Cols(); x++) {
			int xx = x * res / in.Cols();
			if (in[y][x] >= 0) {
				CHECK_LT(in[y][x], 3);
				votes[ in[y][x] ][yy][xx]++;
			}
		}
	}

	for (int y = 0; y < res; y++) {
		for (int x = 0; x < res; x++) {
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





typedef pair<Vector<3>, Vector<3> > FloorSeg;
vector<FloorSeg> floor_segs;



struct WallSegment {
	Vector<3> tl, tr, bl, br;
	int axis;
	WallSegment() {}
	WallSegment(const Vector<3>& tl_,
							const Vector<3>& tr_,
							const Vector<3>& bl_,
							const Vector<3>& br_,
							int axis_)
		: tl(tl_), tr(tr_), bl(bl_), br(br_), axis(axis_) { }
};












void DrawFloorPlan(const string& filename, const Map& map, ImageRef sz) {
	DLOG << "writing floorplan to " << filename;

	double xmin = INFINITY, xmax = -INFINITY;
	double ymin = INFINITY, ymax = -INFINITY;
	BOOST_FOREACH(const FloorSeg& seg, floor_segs) {
		for (int i = 0; i < 2; i++) {
			const Vector<3>& p = i ? seg.second : seg.first;
			if (p[0] < xmin) xmin = p[0];
			if (p[0] > xmax) xmax = p[0];
			if (p[1] < ymin) ymin = p[1];
			if (p[1] > ymax) ymax = p[1];
		}
	}
	const double kMargin = 0.5;
	xmin -= kMargin;
	xmax += kMargin;
	ymin -= kMargin;
	ymax += kMargin;

	DREPORT(xmin, xmax, ymin, ymax);

	double len = max(xmax-xmin, ymax-ymin);

	Vector<2> origin = makeVector(xmin, ymin);
	Matrix<2> scale = Identity * (min(sz.x, sz.y) / len);

	DREPORT(len, origin, scale);

	ImageRGB<byte> canvas(sz);
	canvas.Clear(Colors::white());

	PixelRGB<byte> ptcolor(127,0,0);
	BOOST_FOREACH(const Vector<3>& p, map.pts) {
		DrawSpot(canvas, scale*(p.slice<0,2>()-origin), ptcolor, 2);
	}
	PixelRGB<byte> linecolor(0,127,0);
	BOOST_FOREACH(const FloorSeg& seg, floor_segs) {
		DrawThickLineClipped(canvas, 
												 scale*(seg.first.slice<0,2>()-origin),
												 scale*(seg.second.slice<0,2>()-origin) ,
												 linecolor,
												 2);
	}

	WriteImage(filename, canvas);
}







void GenerateFrameViz(const Frame& fs,
											const vector<WallSegment>& world_model,
											Map& map,
											LeeRecovery& recovery) {
	ptam::ATANCamera& cam = map.undistorter.cam;

	// Read the image
	ImageBundle image(fs.filename);
	string basename = PaddedInt(fs.id, 8);

	// Draw the map points
	ImageRGB<byte> pt_canvas;
	ImageCopy(image.rgb, pt_canvas);
	BOOST_FOREACH(const Vector<3>& pt, map.pts) {
		DrawSpot(pt_canvas, cam.Project(project(fs.CfW*pt)), Colors::blue(), 1);
	}
	WriteImage("out/pt_video/"+basename+".png", pt_canvas);

	// Identify lines
	PeakLines lines;
	DREPORT(fs.CfW.ln());
	lines.Compute(image, fs.CfW, map);
	lines.OutputSceneAxesViz("out/axis_video/"+basename+".png");
	lines.OutputSegmentsViz("out/peakline_video/"+basename+".png");

	// Copy the lines to a different format
	LineSweepInput sweep_input(image);
	for (int j = 0; j < 3; j++) {
		sweep_input.vpts[j] = lines.image_vpts[j];
		BOOST_FOREACH(const LineSeg& peakline, lines.segments[j]) {
			Vector<3> a = unproject(peakline.bin.start);
			Vector<3> b = unproject(peakline.bin.end);
			sweep_input.lines.push_back(LineSweepSegment(a, b, a^b, j));
		}
	}

	// Compute line sweeps
	IsctGeomLabeller labeller;
	labeller.Compute(sweep_input);
	labeller.OutputOrientViz("out/orients_video/"+basename+".png");

	// Project the model
	MatI surfs(image.ny(), image.nx(), recovery.vert_axis);
	BOOST_FOREACH(const WallSegment& wall, world_model) {
		Vector<3> ret_tl = fs.CfW * wall.tl;
		Vector<3> ret_tr = fs.CfW * wall.tr;
		Vector<3> ret_bl = fs.CfW * wall.bl;
		Vector<3> ret_br = fs.CfW * wall.br;
		ClipToPositive(ret_tr, ret_tl);
		ClipToPositive(ret_br, ret_bl);

		Vector<2> im_tl = cam.Project(project(ret_tl));
		Vector<2> im_tr = cam.Project(project(ret_tr));
		Vector<2> im_bl = cam.Project(project(ret_bl));
		Vector<2> im_br = cam.Project(project(ret_br));

		vector<Vector<3> > poly;
		poly.push_back(unproject(im_tl));
		poly.push_back(unproject(im_bl));
		poly.push_back(unproject(im_br));
		poly.push_back(unproject(im_tr));
		int filled = FillPolygonFast(poly, surfs, wall.axis);
	}

	// Draw the projected surfaces
	ImageRGB<byte> surf_canvas;
	ImageCopy(image.rgb, surf_canvas);
	for (int y = 0; y < image.ny(); y++) {
		const int* in_row = surfs[y];
		PixelRGB<byte>* out_row = surf_canvas[y];
		for (int x = 0; x < image.nx(); x++) {
			BlendWith(out_row[x], Colors::primary(in_row[x]), 0.5);
		}
	}
	WriteImage("out/surf_video/"+basename+".png", surf_canvas);
}

















void BuildModel(MapViz& mapviz,
								KeyFrame& kf,
								KeyFrameWidget& kfw,
								Widget3D& container) {
	SCOPED_INDENT;
	Map& map = mapviz.map;
	ImageBundle& image = kf.image;
	ptam::ATANCamera& cam = mapviz.map.undistorter.cam;

	PeakLines lines;
	LeeRecovery recovery(&cam, image.sz());

	IsctGeomLabeller labeller;
	LineSweepInput sweep_input(image);

	vector<LeeEdge> edges[3];
	MatI orient_est;
	int nv = 0, nh = 0;

	double floor_z;
	const LeeBuilding* best_bld;

	TIMED("Complete") {
		// Detect lines
		TIMED("Detect lines") INDENTED lines.Compute(image, kf.CfW, map);
		lines.OutputSegmentsViz("out/lines.png");

		// Copy the lines into line sweeper format
		TIMED("Copy lines");
		for (int i = 0; i < 3; i++) {
			sweep_input.vpts[i] = lines.image_vpts[i];
			BOOST_FOREACH(const LineSeg& peakline, lines.segments[i]) {
				Vector<3> a = unproject(peakline.bin.start);
				Vector<3> b = unproject(peakline.bin.end);
				sweep_input.lines.push_back(LineSweepSegment(a, b, a^b, i));
			}
		}

		// Compute line sweeps
		TIMED("Compute orientations") INDENTED labeller.Compute(sweep_input);

		// Copy the lines into structure recovery format
		int next_id = 0;
		for (int i = 0; i < 3; i++) {
			int ii = 0;
			BOOST_FOREACH(const LineSeg& peakline, lines.segments[i]) {
				Vector<3> a = unit(unproject(cam.UnProject(peakline.bin.start)));
				Vector<3> b = unit(unproject(cam.UnProject(peakline.bin.end)));
				edges[i].push_back(LeeEdge(a, b, i, next_id++));
			}
		}

		DREPORT(edges[0].size());
		DREPORT(edges[1].size());
		DREPORT(edges[2].size());

		// Enumerate building hypotheses
		TIMED("Initialize buildings") INDENTED recovery.Initialize(edges, kf.CfW);
		if (recovery.buildings.empty()) {
			DLOG << "No valid initializations";
			return;
		}
		TIMED("Enumerate buildings") INDENTED recovery.Enumerate();
		DREPORT(recovery.buildings.size());

		// Down-size the orientation map
		DownsampleOrients(labeller.orient_map, orient_est, *gvOrientRes);

		// Identify the best building
		best_bld = &recovery.buildings[4348];
		/*int best_index, best_score = -1;
		TIMED("Select best") COUNTED_FOREACH(int i, const LeeBuilding& bld, recovery.buildings) {
			int score = recovery.ScoreBuilding(bld, orient_est);
			if (score > best_score) {
				best_score = score;
				best_index = i;
				best_bld = &bld;
			}
		}
		DREPORT(best_score);
		DREPORT(best_index);*/


		// Locate the floor by looking for the n-th percentile in Z
		vector<double> zs;
		BOOST_FOREACH(const Vector<3>& p, map.pts) {
			zs.push_back(p[2]);
		}
		sort_all(zs);
		floor_z = zs[zs.size()*0.98];
	}


	// Vizualize the full orientations
	ImageRGB<byte> orient_canvas;
	ImageCopy(image.rgb, orient_canvas);
	labeller.DrawOrientViz(orient_canvas);
	lines.DrawSegments(orient_canvas);
	WriteImage("out/orients.png", orient_canvas);

	// Vizualize downsampled orientations
	ImageRGB<byte> est_canvas;
	DrawOrientations(orient_est, est_canvas, image.sz());
	WriteImage("out/est_orients.png", est_canvas);

	// Vizualize the building
	recovery.OutputAllViz(image.rgb, *best_bld, "out/best");





	// Reconstruct the 3D model
	vector<WallSegment> world_model;
	PixelRGB<byte> ceil_color = Colors::primary(recovery.vert_axis);
	PixelRGB<byte> floor_color = Colors::primary(recovery.vert_axis);

	// Build the 3D model
	LeeBuilding::ConstCnrIt left = best_bld->cnrs.begin();
	LeeBuilding::ConstCnrIt right = successor(left);
	while (right != best_bld->cnrs.end()) {
		Vector<3> bl = recovery.FloorPoint(left->right_floor, floor_z);
		Vector<3> br = recovery.FloorPoint(right->left_floor, floor_z);
		Vector<3> tl = recovery.CeilPoint(left->right_ceil, bl);
		Vector<3> tr = recovery.CeilPoint(right->left_ceil, br);

		if (bl[2] < 0) {
			bl = -bl;
			br = -br;
			tl = -tl;
			tr = -tr;
		}

		Vector<3> world_tl = kf.WfC * tl;
		Vector<3> world_tr = kf.WfC * tr;
		Vector<3> world_bl = kf.WfC * bl;
		Vector<3> world_br = kf.WfC * br;

		// Color of this wall
		int wall_axis = recovery.OtherHorizAxis(left->right_axis);
		floor_segs.push_back(make_pair(world_bl, world_br));
		world_model.push_back(WallSegment(world_tl, world_tr, world_bl, world_br, wall_axis));

		PixelRGB<byte> wall_color = Colors::primary(wall_axis);

		// The camera center on the floor and ceiling
		Vector<3> floor_c = kf.WfC.get_translation();
		Vector<3> ceil_c = kf.WfC.get_translation();
		floor_c[2] = floor_z;
		ceil_c[2] = world_tl[2];

		// Add the wall segment
		QuadWidget* qwall = new QuadWidget(world_tl, world_tr, world_br, world_bl);
		qwall->color = wall_color;
		container.AddOwned(qwall);

		// Add the floor tri
		QuadWidget* qfloor = new QuadWidget(world_bl, world_br, floor_c, floor_c);
		qfloor->color = floor_color;
		container.AddOwned(qfloor);

		// Add the ceil tri
		QuadWidget* qceil = new QuadWidget(world_tl, world_tr, ceil_c, ceil_c);
		qceil->color = ceil_color;
		container.AddOwned(qceil);

		left++;
		right++;
	}




	// Project the model into each frame
	vector<int> reproj_ids;
	ParseMultiRange(GV3::get<string>("LeeRecovery.ReprojFrames"), reproj_ids);
	COUNTED_FOREACH (int i, int id, reproj_ids) {
		if (id < 0 || id >= map.frame_specs.size()) {
			DLOG << "Frame " << id << " out of range";
		} else {
			DLOG << "Vizualizing frame " << (id+1)
					 << " (" << i << " of " << reproj_ids.size() << ")";
			INDENTED TIMED("Render")
				GenerateFrameViz(map.frame_specs[id], world_model, map, recovery);
		}
	}
}












int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);

	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map and rotate to canonical image
	MapViz mapviz;
	//Vector<3> lnR = makeVector(1.58235, -0.0309467, 0.0443291);  // for eng_corridor
	Vector<3> lnR = makeVector(1.1531, 1.26237, -1.24435);  // for lab_kitchen1
	//Vector<3> lnR = makeVector(2.86042, -1.21951, 0.0804226);  // for princes_kitchen
	mapviz.map.scene_to_slam = SO3<>::exp(lnR);
	mapviz.map.Rotate(mapviz.map.scene_to_slam.inverse());

	// TODO: map.Load() should do this automatically
	mapviz.map.undistorter.Compute(ImageRef(640,480));

	Widget3D container;
	mapviz.viewer.Add(container);

	COUNTED_FOREACH(int i, KeyFrameWidget* kfw, mapviz.kf_widgets) {
		kfw->DoubleClick.add(bind(&BuildModel, 
															ref(mapviz),
															ref(mapviz.map.kfs[i]),
															ref(*kfw),
															ref(container)));
	}


	mapviz.viewer.BindKey('w', bind(&DrawFloorPlan,
																	"out/floorplan.png",
																	ref(mapviz.map),
																	ImageRef(640,480)));

	mapviz.viewer.BindKey('c', bind(&Widget3D::ClearChildren, ref(container)));

	int ref_i = -1;
	COUNTED_FOREACH(int i, KeyFrame& kf, mapviz.map.kfs) {
		if (kf.id == GV3::get<int>("LeeRecovery.RefFrame")) {
			ref_i = i;
			break;
		}
	}

	if (ref_i == -1) {
		DLOG << "Reference keyframe " << GV3::get<int>("LeeRecovery.RefFrame") << " not found";
	} else {
		BuildModel(mapviz, mapviz.map.kfs[ref_i], *mapviz.kf_widgets[ref_i], container);
	}

	mapviz.Run();
}
