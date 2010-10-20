#include <boost/array.hpp>
#include <LU.h>

#include "entrypoint_types.h"
#include "map.h"
#include "colors.h"
#include "canvas.h"
#include "timer.h"
#include "manhattan_dp.h"
#include "stereo_payoffs.h"

#include "integral_col_image.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"
#include "math_utils.tpp"
#include "vector_utils.tpp"

using namespace indoor_context;


void OutputTransformedImage(const string& file,
														const ImageRGB<byte>& image,
														const Mat3& h,
														Vec2I bounds) {
	Mat3 hinv = LU<3>(h).get_inverse();
	ImageRGB<byte> canvas(bounds[0], bounds[1]);
	for (int y = 0; y < bounds[1]; y++) {
		PixelRGB<byte>* row = canvas[y];
		for (int x = 0; x < bounds[0]; x++) {
			ImageRef p = asIR(project(hinv * makeVector(x,y,1.0)));
			if (p.x >= 0 && p.x < image.GetWidth() &&
					p.y >= 0 && p.y < image.GetHeight()) {
				row[x] = image[p];
			} else {
				row[x] = Colors::white();
			}
		}
	}
	WriteImage(file, canvas);
}





PosedImage& GetFrame(Map& map, int id) {
	Frame* f = map.KeyFrameByIdOrDie(id);
	f->LoadImage();
	PosedImage& im = f->image;
	im.BuildMono();
	return im;
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 4) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro FRAME1 AUX_FRAMES";
		return -1;
	}

	// Input arguments
	const char* path = argv[1];
	const int base_id = atoi(argv[2]);
	const vector<int> aux_ids = ParseMultiRange<int>(argv[3]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(path, gt_map);

	// Get the floor and ceiling positions
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();
	Vec3 vup = map.kfs[0].pc->pose_inverse() * makeVector(0,1,0);
	if (Sign(zceil-zfloor) == Sign(vup[2])) {
		swap(zfloor, zceil);
	}

	// Get base frame
	const PosedImage& base_image = GetFrame(map, base_id);
	Mat3 fToC = GetManhattanHomology(base_image.pc(), zfloor, zceil);
	DPGeometry geom(&base_image.pc(), fToC);

	// Process each auxiliary frame
	int n_aux = 0;
	StereoPayoffs payoff_gen;
	boost::array<MatF,2> joint_payoffs;
	joint_payoffs[0].Resize(geom.grid_size[1], geom.grid_size[0], 0);  // must initialize to zero
	joint_payoffs[1].Resize(geom.grid_size[1], geom.grid_size[0], 0);
	BOOST_FOREACH(int aux_id, aux_ids) {
		if (aux_id == base_id) continue;
		TITLE("Computing payoffs w.r.t to aux frame " << aux_id);
		payoff_gen.Compute(base_image, GetFrame(map, aux_id), geom, zfloor, zceil);
		joint_payoffs[0] += payoff_gen.payoffs[0];
		joint_payoffs[1] += payoff_gen.payoffs[1];
		n_aux++;
	}
	joint_payoffs[0] /= n_aux;
	joint_payoffs[1] /= n_aux;

	// Do reconstruction
	ManhattanDPReconstructor recon;
	recon.Compute(base_image, geom, joint_payoffs);

	// Compare results with ground truth
	double acc = recon.GetAccuracy(gt_map.floorplan());
	DLOG << format("Accuracy: %.2f%%") % (acc*100.0);

	/*payoff_gen.OutputPayoffs("out/payoffs.png");
	payoff_gen.OutputRawPayoffs("out/raw_payoffs.png");
	WriteMatrixImageRescaled("out/vrect_l.png", payoff_gen.l_vrect_im_tr.Transpose());
	WriteMatrixImageRescaled("out/vrect_r.png", payoff_gen.r_vrect_im_tr.Transpose());*/

	recon.OutputSolutionOrients("out/soln.png");
	recon.OutputGridViz("out/grid_soln.png");
	OutputTransformedImage("out/grid.png", base_image.rgb, geom.imageToGrid, geom.grid_size);

	ImageRGB<byte> raw_payoffs;
	MatrixToImageRescaled(payoff_gen.payoffs[0], raw_payoffs);
	recon.dp.DrawGridSolution(raw_payoffs);
	WriteImage("out/soln_payoffs.png", raw_payoffs);

	return 0;
}
