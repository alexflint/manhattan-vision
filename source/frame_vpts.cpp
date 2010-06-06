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

#include "line_detector.h"
#include "vanishing_points.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;

const lazyvar<double> gvSigma("LineModel.LogLikSigma");
const lazyvar<double> gvSpurious("LineModel.SpuriousLogLik");

template <typename T>
void mydelete(T* x) {
	delete x;
}

// Compute likelihood for a segment arising from a vanishing point
double SegmentLogLik(const LineSegment& seg, const Vector<3>& vpt) {
	// We compute the distance from the end points to the line that
	// passes through the segment's midpoint and the vanishing point.
	const Vector<3> midp = unproject((seg.start + seg.end) / 2);
	const double sigma = *gvSigma;
	const double dist = EuclPointLineDist(unproject(seg.start), vpt^midp);
	return -dist*dist / (2.0 * sigma * sigma);
}

void DrawSceneAxes(ImageRGB<byte>& canvas,
									 const SE3<>& scene_to_retina,
									 ptam::ATANCamera& cam) {
	Vector<2> image_ctr = cam.Project(makeVector(0,0));
	Vector<3> scene_ctr = scene_to_retina.inverse() * makeVector(0,0,1);
	for (int j = 0; j < 3; j++) {
		Vector<3> retina_ej = scene_to_retina * (scene_ctr + GetAxis<3>(j));
		Vector<2> image_ej = cam.Project(project(retina_ej));
		ClipLineToImage(image_ctr, image_ej, canvas.GetSize());
		DrawLine(canvas, image_ctr, image_ej, BrightColors::Get(j));
	}
}


void OutputSceneAxesViz(const string& filename,
												const ImageBundle& image,
												const SE3<>& scene_to_retina,
												ptam::ATANCamera& cam) {
	ImageRGB<byte> canvas;
	ImageCopy(image.rgb, canvas);
	DrawSceneAxes(canvas, scene_to_retina, cam);
	WriteImage(filename, canvas);
}

void DrawLineAssoc(ImageRGB<byte>& canvas,
									 const VanishingPoints& vpts) {
	for (int j = 0; j < vpts.lines.segments.size(); j++) {
		int owner = vpts.detector.owners[j];
		if (owner >= 0) {
			const LineSegment& seg = vpts.lines.segments[j];
			Vector<2> midp = (seg.start + seg.end) / 2.0;
			Vector<2> vpt = project(vpts.detector.vanpts[owner].pt);
			ClipLineToImage(midp, vpt, canvas.GetSize());
			DrawLine(canvas, midp, vpt, BrightColors::Get(owner));
		}
	}
	vpts.lines.Draw(canvas, vpts.detector.owners, makeVector(0.0, 0.0));
}

void OutputLineAssocViz(const string& filename,
												const ImageBundle& image,
												const VanishingPoints& vpts) {
	ImageRGB<byte> canvas;
	ImageCopy(image.rgb, canvas);
	DrawLineAssoc(canvas, vpts);
	WriteImage(filename, canvas);
}


int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map and rotate to canonical frame
	Map map;
	map.load_images = false;
	map.Load();
	ptam::ATANCamera& cam = map.undistorter.cam;

	Vector<3> lnR = makeVector(1.1531, 1.26237, -1.24435);  // for lab_kitchen1
	map.scene_to_slam = SO3<>::exp(lnR);

	SE3<> scene_to_slam;
	scene_to_slam.get_rotation() = map.scene_to_slam;

	IsctGeomLabeller labeller;
	VanishingPoints vpts;  // will store data in here
	Gradients gradients;

	const int kNumWorkers = 6;
	Worker workers[kNumWorkers];

	for (int i = 0; i < map.frame_specs.size(); i++) {
		const Frame& fs = map.frame_specs[i];
		TIMED("Complete") if (!fs.lost) {
			DREPORT(i);

			SE3<> scene_to_retina = fs.pose * scene_to_slam;

			// Load the image
			ImageBundle* frame = new ImageBundle(fs.filename);;
			string basename = "out/frame"+PaddedInt(i,4);

			// Project vanishing points into the image
			vpts.detector.vanpts.resize(3);
			for (int j = 0; j < 3; j++) {
				Vector<3> retina_vpt = atretina(fs.pose.get_rotation() * (map.scene_to_slam * GetAxis<3>(j)));
				vpts.detector.vanpts[j].pt = unproject(cam.Project(project(retina_vpt)));
			}

			string filename = basename+"_vpts.png";
			workers[i%kNumWorkers].Wait();  // don't let the
			workers[i%kNumWorkers].Add
				(bind(&OutputSceneAxesViz, filename, ref(*frame), scene_to_retina, cam));
			workers[i%kNumWorkers].Add(bind(&mydelete<ImageBundle>, frame));


			// Compute image gradients
			/*frame.BuildMono();
			gradients.Compute(frame.mono);
			ImageRGB<byte> canvas;
			ImageCopy(frame.rgb, canvas);
			const double kMagThresh = pow(GV3::get<double>("LinesAfterVpts.MagThresh"), 2.0);
			// Watch out here, "x" and "y" have three seperate meanings!
			TIMED("Associate pixels") 
			for (int y = 0; y < frame.ny(); y++) {
				const PixelF* dxrow = gradients.diffx[y];
				const PixelF* dyrow = gradients.diffy[y];
				const float* magrow = gradients.magnitude_sqr[y];
				for (int x = 0; x < frame.nx(); x++) {
					if (magrow[x] > kMagThresh) {
						Vector<2> p = makeVector(x, y);
						double norm = sqrt(magrow[x]);
						Vector<2> tangent = makeVector(dyrow[x].y/norm, -dxrow[x].y/norm);
						Vector<3> endpt = unproject(p+tangent);
						double dmin = 0.2;
						int jmin = -1;
						for (int j = 0; j < 3; j++) {
							Vector<3> line = unproject(p)^vpts.detector.vanpts[j].pt;
							double d = EuclPointLineDist(endpt, line);
							if (d < dmin) {
								dmin = d;
								jmin = j;
							}
						}

						if (jmin == -1) {
							canvas[y][x] = PixelRGB<byte>(255,255,255);
						} else {
							canvas[y][x] = BrightColors::Get(jmin);
						}
					}
				}
			}
			WriteImage(basename+"_pixels.png", canvas);*/


			// Detect lines and assign them to vanishing points
			/*TIMED("Detect lines") vpts.lines.Compute(frame);
			vpts.detector.owners.Resize(vpts.lines.segments.size());
			TIMED("Associate lines") for (int j = 0; j < vpts.lines.segments.size(); j++) {
				double best_loglik = *gvSpurious;
				vpts.detector.owners[j] = -1;
				for (int k = 0; k < 3; k++) {
					double loglik = SegmentLogLik(vpts.lines.segments[j],
																				vpts.detector.vanpts[k].pt);
					if (loglik > best_loglik) {
						vpts.detector.owners[j] = k;
						best_loglik = loglik;
					}
				}
			}

			OutputLineAssocViz(basename+"_lines.png", frame, vpts);

			// Draw the line sweeps
			TIMED("Compute orientation labels") INDENTED labeller.Compute(frame, vpts);
			//labeller.sweeper.OutputSupportViz(basename+"_support");
			//labeller.OutputOrientViz(basename+"_orients.png");
			*/

		}
	}

	DLOG << "waiting for workers to finish";
	for (int i = 0; i < kNumWorkers; i++) {
		workers[i].Join();
	}

	return 0;
}
