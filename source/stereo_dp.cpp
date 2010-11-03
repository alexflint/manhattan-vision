#include <boost/array.hpp>
#include <LU.h>

#include "entrypoint_types.h"
#include "map.h"
#include "colors.h"
#include "canvas.h"
#include "timer.h"
#include "manhattan_dp.h"
#include "stereo_payoffs.h"
#include "geom_utils.h"
#include "bld_helpers.h"
#include "canny.h"

#include "integral_col_image.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"
//#include "numeric_utils.tpp"
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





void MagToImage(const MatF& m, ImageF& q) {
	ResizeImage(q, m.Cols(), m.Rows());
	for (int y = 0; y < m.Rows(); y++) {
		const float* inrow = m[y];
		PixelF* outrow = q[y];
		for (int x = 0; x < m.Cols(); x++) {
			outrow[x].y = sqrt(inrow[x]);
		}
	}
}

void MakeGradientImage(const PosedImage& orig,
											 Gradients& container,
											 PosedImage& out) {
	out.pc().SetPose(orig.pc().pose());
	out.pc().SetCamera(&orig.pc().camera());
	ImageCopy(orig.rgb, out.rgb);
	container.Compute(orig);
	MagToImage(container.magnitude_sqr, out.mono);
	out.SaveMono();
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 4) {
		DLOG << "Usage: "<<argv[0]<<" SEQUENCE TEST_IDS REL_AUX_IDS";
		DLOG << "  example: "<<argv[0]<<" truthed_map.pro 5:5:50 -3:3";
		DLOG << "           proceses frame 5,10,...,50 with +/-3 frame supporting";
		return -1;
	}

	// Input arguments
	const char* sequence = argv[1];
	const vector<int> test_ids = ParseMultiRange<int>(argv[2]);
	const vector<int> rel_aux_ids = ParseMultiRange<int>(argv[3]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	// Get the floor and ceiling positions
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();
	Vec3 vup = map.kfs[0].pc->pose_inverse() * makeVector(0,1,0);
	if (Sign(zceil-zfloor) == Sign(vup[2])) {
		swap(zfloor, zceil);
	}

	// The payoff generator
	Gradients gradients;
	StereoPayoffs payoff_gen;
	boost::array<MatF,2> stereo_payoffs;

	// Do the reconstructions
	double sum_acc = 0;
	format filepat("out/frame%02d_%s.png");
	BOOST_FOREACH(int base_id, test_ids) {
		TITLE("Reconstructing frame " << base_id);

		// Get base frame
		const PosedImage& base_image = map.ImageByIdOrDie(base_id);
		DPGeometry geom(&base_image.pc(), zfloor, zceil);

		// Compute gradients
		PosedImage base_gradients;
		MakeGradientImage(base_image, gradients, base_gradients);

		// Process each auxiliary frame
		int n = 0;
		stereo_payoffs[0].Resize(geom.grid_size[1], geom.grid_size[0], 0);  // must initialize to zero
		stereo_payoffs[1].Resize(geom.grid_size[1], geom.grid_size[0], 0);
		BOOST_FOREACH(int d, rel_aux_ids) {
			int aux_id = base_id+d;
			if (aux_id == base_id) continue;
			PosedImage aux_gradients;
			MakeGradientImage(map.ImageByIdOrDie(aux_id), gradients, aux_gradients);
			TITLE("Computing payoffs w.r.t. aux frame " << aux_id);
			payoff_gen.Compute(base_gradients, aux_gradients, geom, zfloor, zceil);
			stereo_payoffs[0] += payoff_gen.payoffs[0];
			stereo_payoffs[1] += payoff_gen.payoffs[1];
			n++;
		}
		stereo_payoffs[0] /= n;
		stereo_payoffs[1] /= n;

		// Do reconstruction
		ManhattanDPReconstructor recon;
		recon.Compute(base_image, geom, stereo_payoffs);

		// Compare results with ground truth
		double acc = recon.GetAccuracy(gt_map.floorplan());
		DLOG << format("Accuracy: %.2f%%") % (acc*100.0);
		sum_acc += acc;

		MatI gt;
		GetTrueOrients(gt_map.floorplan(), base_image.pc(), gt);
		WriteOrientationImage(str(filepat % base_id % "gt"), gt);

		recon.OutputSolutionOrients(str(filepat % base_id % "soln"));
		//recon.OutputGridViz("out/grid_soln.png");
		//OutputTransformedImage("out/grid.png", base_image.rgb, geom.imageToGrid, geom.grid_size);

		/*ImageRGB<byte> raw_payoffs;
			MatrixToImageRescaled(payoff_gen.payoffs[0], raw_payoffs);
			recon.dp.DrawGridSolution(raw_payoffs);
			WriteImage("out/soln_payoffs.png", raw_payoffs);*/
	}

	double ap = sum_acc / test_ids.size();
	DLOG << format("Average accuracy: %.2f%%") % (ap*100.0);
	
	return 0;
}
