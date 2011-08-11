#include <iomanip>

#include <LU.h>

#include "common_types.h"
#include "map_widgets.h"
#include "map.h"
#include "textons.h"
#include "vars.h"
#include "viewer3d.h"
#include "line_sweeper.h"
#include "gl_utils.tpp"
#include "timer.h"
#include "geom_utils.h"
#include "worker.h"
#include "clipping.h"

#include "guided_line_detector.h"

#include "line_detector.h"
#include "vanishing_points.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
//#include "numeric_utils.tpp"

using namespace indoor_context;
using namespace toon;

const lazyvar<double> gvSigma("LineModel.LogLikSigma");
const lazyvar<double> gvSpurious("LineModel.SpuriousLogLik");

// Compute likelihood for a segment arising from a vanishing point
double SegmentLogLik(const LineSeg& seg, const Vector<3>& vpt) {
	// We compute the distance from the end points to the line that
	// passes through the segment's midpoint and the vanishing point.
	const Vector<3> midp = unproject((project(seg.start) + project(seg.end)) / 2);
	const double sigma = *gvSigma;
	const double dist = EuclPointLineDist(seg.start, vpt^midp);
	return -dist*dist / (2.0 * sigma * sigma);
}

void DrawLineAssoc(ImageRGB<byte>& canvas,
									 const vector<LineDetection>& detections,
									 const Vector<3> vpts[3]) {
	BOOST_FOREACH(const LineDetection& det, detections) {
		if (det.axis >= 0) {
			Vector<2> midp = (project(det.seg.start) + project(det.seg.end)) / 2.0;
			DrawLineClipped(canvas, midp, project(vpts[det.axis]), Colors::primary(det.axis));
		}
		det.Draw(canvas, Colors::primary(det.axis));
	}
}

void OutputLineAssocViz(const string& filename,
												const ImageBundle& image,
												const vector<LineDetection>& detections,
												const Vector<3> vpts[3]) {
	ImageRGB<byte> canvas;
	ImageCopy(image.rgb, canvas);
	DrawLineAssoc(canvas, detections, vpts);
	WriteImage(filename, canvas);
}

void DrawHistogram(ImageRGB<byte>& canvas,
									 const VecI& hist,
									 float aspect) {
	int nx = hist.Size();
	int ny = roundi(aspect*nx);
	ResizeImage(canvas, nx, ny);
	int maxv = hist.MaxValue();
	PixelRGB<byte> white(255,255,255), black(0,0,0);
	for (int y = 0; y < ny; y++) {
		int cutoff = (ny-y)*maxv/ny;
		PixelRGB<byte>* row = canvas[y];
		for (int x = 0; x < nx; x++) {
			row[x] = (hist[x] >= cutoff) ? black : white;
		}
	}
}


void Process(Map& map, int frame, GuidedLineDetector& line_detector) {
	DREPORT(frame);
	SCOPED_INDENT;

	// Load the image
	const Frame& fs = map.frame_specs[frame];
	ImageBundle image(fs.filename);

	LineSweepInput sweep_input(image);
	IsctGeomLabeller labeller;

	SE3<> rot;
	rot.get_rotation() = map.scene_from_slam.inverse();
	SE3<> pose = fs.CfW * rot;

	TIMED("Complete") {
		// Detect liens
		TIMED("Detect lines") INDENTED line_detector.Compute(image, pose, map);

		// Copy the lines into a different format
		TIMED("Copy lines");
		for (int i = 0; i < 3; i++) {
			sweep_input.vpts[i] = line_detector.image_vpts[i];
			BOOST_FOREACH(const LineDetection& det, line_detector.detections[i]) {
				sweep_input.lines.push_back(det);
			}
		}

		// Compute line sweeps
		TIMED("Compute orientations") INDENTED labeller.Compute(sweep_input);
	}

	string basename = "out/image"+PaddedInt(frame,4);

	//labeller.sweeper.OutputSupportViz(basename+"_support");
	line_detector.OutputSegmentsViz(basename+"_lines.png");
	line_detector.OutputRayViz(basename+"_rays.png");
	//line_detector.OutputProjViz(basename+"_proj");
	line_detector.OutputSceneAxesViz(basename+"_axes.png");
	//line_detector.OutputSegPixelsViz(basename+"_peak_pixels.png");
	//line_detector.OutputAssociationViz(basename+"_assoc.png");
	//line_detector.OutputBinViz(basename+"_bins.png");
	//line_detector.OutputThetaViz(basename+"_thetas");
	//line_detector.OutputDistsViz(basename+"_dists.png");
	//line_detector.OutputVptWindowViz(basename+"_windows.png");

	ImageRGB<byte> orient_canvas;
	ImageCopy(image.rgb, orient_canvas);
	labeller.DrawOrientViz(orient_canvas);
	line_detector.DrawSegments(orient_canvas);
	WriteImage(basename+"_orients.png", orient_canvas);
}


int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro";
		return 0;
	}

	// Load the truthed map
	proto::TruthedMap tru_map;
	ifstream s(argv[1], ios::binary);
	CHECK(tru_map.ParseFromIstream(&s));

	// Load the map
	Map map;
	map.auto_undistort = false;
	map.kf_ids_to_load.clear();
	BOOST_FOREACH(const proto::TruthedFrame& cur, tru_map.frame()) {
		map.kf_ids_to_load.push_back(cur.id());
	}
	map.LoadXml(tru_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));

	// Initialize the reconstructor
	recon.reset(new ManhattanReconstruction);

	// Begin the processing
	//INDENTED ProcessFrame(5, map, tru_map);
	//INDENTED ProcessFrame(1, map, tru_map);
	//INDENTED ProcessFrame(7, map, tru_map);

	// Process each frame
	for (int i = 0; i < tru_map.frame_size(); i++) {
		INDENTED ProcessFrame(i, map, tru_map);
	}

	return 0;
}


int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map and rotate to canonical image
	Map map;
	map.load_images = false;
	map.Load();
	//map.RotateToSceneFrame();

	Vector<3> lnR = makeVector(1.1531, 1.26237, -1.24435);  // for lab_kitchen1
	//Vector<3> lnR = makeVector(-1.47651 -0.526332 -0.108577);  // for princes_kitchen
	map.RotateToSceneFrame(SO3<>::exp(lnR).inverse());
	//map.Rotate(map.scene_to_slam.inverse());

	const int kNumWorkers = 1;  // increase this for efficiency but broken logging
	Worker workers[kNumWorkers];
	ImageBundle images[kNumWorkers];
	GuidedLineDetector peak_lines[kNumWorkers];

	map.undistorter.Compute(ImageRef(640,480));

	TIMED("Compute undistort maps")
	for (int i = 0; i < kNumWorkers; i++) {
		peak_lines[i].ComputeUndist(ImageRef(640,480), map.camera);
	}

	vector<int> frame_ids;
	ParseMultiRange(GV3::get<string>("FrameLineSweeper.Frames"), frame_ids);

	for (int i = 0; i < frame_ids.size(); i++) {
		int id = frame_ids[i];
		if (id < 0 || id >= map.frame_specs.size()) {
			DLOG << "Frame index " << id << " out of range"
					 << " (there are " << map.frame_specs.size() << " frames available)";
		} else {
			const Frame& fs = map.frame_specs[frame_ids[i]];
			if (!fs.lost) {
				int w = i%kNumWorkers;
				workers[w].Add(bind(&Process, ref(map), frame_ids[i], ref(peak_lines[w])));
			}
		}
	}

	for (int i = 0; i < kNumWorkers; i++) {
		workers[i].Join();
	}
}
