#include <iomanip>
#include <fstream>
#include <iterator>
#include <set>

#include <LU.h>

#include "common_types.h"
#include "map.h"
#include "vars.h"
#include "line_sweeper.h"
#include "timer.h"
#include "geom_utils.h"
#include "worker.h"
#include "clipping.h"

#include "guided_line_detector.h"

#include "line_detector.h"
#include "vanishing_points.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace toon;

Vector<3> positive(const Vector<3>& v, int axis) {
	return v[axis] >= 0 ? v : -v;
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map and rotate to canonical image
	Map map;
	map.load_images = false;
	map.Load();

	Vector<3> lnR = makeVector(1.1531, 1.26237, -1.24435);  // for lab_kitchen1

	SE3<> scene_to_slam;
	scene_to_slam.get_rotation() = SO3<>::exp(lnR);

	map.scene_to_slam = SO3<>::exp(lnR);

	// TODO: map.Load() should do this automatically
	map.undistorter.Compute(ImageRef(640,480));
	ptam::ATANCamera& cam = map.undistorter.cam;

	vector<int> frame_ids;
	ParseMultiRange(GV3::get<string>("CanonicalImages.InputFrames"), frame_ids);

	COUNTED_FOREACH(int i, int id, frame_ids) {
		if (id < 0 || id >= map.frame_specs.size()) {
			DLOG << "Frame index " << id << " out of range"
					 << " (there are " << map.frame_specs.size() << "frames available)";
		} else {
			const Frame& fs = map.frame_specs[frame_ids[i]];
			if (!fs.lost) {


				// Read the image and detect lines
				ImageBundle image(fs.filename);

				PeakLines lines;
	 			lines.Compute(image, fs.pose, map);
				
				SE3<> pose = fs.pose * scene_to_slam;


				// Project vanishing points
				Vector<3> vpts[3];
				int vert_axis = 0;
				for (int i = 0; i < 3; i++) {
					vpts[i] = pose.get_rotation().get_matrix().T()[i];
					if (abs(vpts[i][1]) > abs(vpts[vert_axis][1])) {
						vert_axis = i;
					}
				}

				int l_axis = (vert_axis+1)%3;
				int r_axis = (vert_axis+2)%3;

				// Compute the rotations
				Matrix<3> R_l_inv, R_r_inv;

				R_l_inv.T()[0] = positive(vpts[l_axis], 0);
				R_l_inv.T()[1] = positive(vpts[vert_axis], 1);
				R_l_inv.T()[2] = vpts[r_axis];

				R_r_inv.T()[0] = positive(vpts[r_axis], 0);
				R_r_inv.T()[1] = positive(vpts[vert_axis], 1);
				R_r_inv.T()[2] = vpts[l_axis];

				Matrix<3> R_l = LU<3>(R_l_inv).get_inverse();
				Matrix<3> R_r = LU<3>(R_r_inv).get_inverse();

				// Warp the images
				ImageRGB<byte> canvas_l(image.sz()), canvas_r(image.sz());
				canvas_l.Clear(PixelRGB<byte>(255,255,255));
				canvas_r.Clear(PixelRGB<byte>(255,255,255));
				for (int y = 0; y < image.ny(); y++) {
					for (int x = 0; x < image.nx(); x++) {

						Vector<3> dest = unproject(cam.UnProject(makeVector(x,y)));

						Vector<3> l_src = R_l_inv * dest;
						Vector<3> r_src = R_r_inv * dest;

						Vector<2> l_im = cam.Project(project(l_src));
						Vector<2> r_im = cam.Project(project(r_src));

						ImageRef l_ir = round_pos(l_im);
						ImageRef r_ir = round_pos(r_im);

						if (image.contains(l_ir)) {
							canvas_l[y][x] = image.rgb[l_ir];
						}
						if (image.contains(r_ir)) {
							canvas_r[y][x] = image.rgb[r_ir];
						}
					}
				}

				ImageRGB<byte> canvas;
				ImageCopy(image.rgb, canvas);

				// Read ratios
				double c_over_f = GV3::get<double>("CanonicalImage.LeftCOverF");
				double f_over_c = 1.0 / c_over_f;

				// Transfer some points
				int horizon_y = image.ny()/2;
				for (int j = 0; j < 50; j++) {
					Vector<2> im_p1 = makeVector(rand()%image.nx(), rand()%image.ny());
					Vector<2> ret_p1 = cam.UnProject(im_p1);

					double y2;
					if (ret_p1[1] > 0) {
						y2 = -c_over_f * ret_p1[1];
					} else {
						y2 = -f_over_c * ret_p1[1];
					}

					Vector<2> ret_p2 = makeVector(ret_p1[0], y2);
					Vector<2> im_p2 = cam.Project(ret_p2);

					Vector<3> orig_ret_p1 = R_l * unproject(ret_p1);
					Vector<3> orig_ret_p2 = R_l * unproject(ret_p2);

					Vector<2> orig_im_p1 = cam.Project(project(orig_ret_p1));
					Vector<2> orig_im_p2 = cam.Project(project(orig_ret_p2));

					DrawLineClipped(canvas, orig_im_p1, orig_im_p2, BrightColors::Get(j));
					DrawLineClipped(canvas_l, im_p1, im_p2, BrightColors::Get(j));
				}

				string basename = "out/canon"+PaddedInt(id, 4);
				WriteImage(basename+"_left.png", canvas_l);
				WriteImage(basename+"_right.png", canvas_r);
				WriteImage(basename+"_orig.png", canvas);
			}
		}
	}
}
