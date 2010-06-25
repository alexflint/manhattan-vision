#include "common_types_entry.h"
#include "guided_line_detector.h"
#include "manhattan_dp.h"
#include "vars.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"
#include "timer.h"
#include "clipping.h"
#include "bld_helpers.h"
#include "safe_stream.h"

#include "histogram.tpp"
#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

const int kNumColorBins = 10;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro";
		return 0;
	}

	// Load the truthed map
	proto::TruthedMap gt_map;
	ifstream s(argv[1], ios::binary);
	CHECK(gt_map.ParseFromIstream(&s)) << "Failed to read from " << argv[1];

	// Load the map
	Map map;
	map.LoadXml(gt_map.spec_file());
	map.RotateToSceneFrame(SO3<>::exp(asToon(gt_map.ln_scene_from_slam())));
	// TODO: load the relevant images

	// Initialize the histograms
	RgbLayout layout(kNumColorBins);
	Histogram<Vec6, RgbLayout> map_hist[3];
	for (int i = 0; i < 3; i++) {
		map_hist[i].Configure(layout);
	}

	vector<MatI> gt_orient_maps, gt_surf_maps, bin_maps;
	vector<Vec3I> gt_counts;

	// Build the model
	BOOST_FOREACH(const proto::TruthedFrame& gt_frame, gt_map.frame()) {
		// Pull out the frame
		const KeyFrame* kf = map.KeyFrameById(gt_frame.id());
		if (kf == NULL) {
			DLOG << "Warning: key frame " << gt_frame.id() << " not found in the map";
			continue;
		}
		ImageRef sz = kf->image.sz();

		// Sign of this equation is arbitrary (but consistent across all frames)
		Vec3 horizon = kf->pc->GetRetinaVpt(2);

		// Get ground truth orientations
		gt_orient_maps.push_back(MatI());
		MatI& gt_orients = gt_orient_maps.back();
		LoadTrueOrients(gt_frame, gt_orients);


		// The true orientations represent surface normals, but here we're interested in
		// ceiling/wall/floor since these are more likely to have distinctive appearances.
		gt_surf_maps.push_back(MatI());
		MatI& gt_surfs = gt_surf_maps.back();
		gt_surfs.Resize(sz.y, sz.x);

		Vec3I gt_count = Zeros;
		for (int y = 0; y < sz.y; y++) {
			int* orientrow = gt_orients[y];
			int* surfrow = gt_surfs[y];
			for (int x = 0; x < sz.x; x++) {
				if (orientrow[x] == 2) {
					Vec3 p = kf->pc->camera.ImToRet(makeVector(1.0*x,1.0*y,1.0));
					surfrow[x] = p*horizon*HalfSign(p[2]) > 0 ? 1 : 2;
				} else {
					surfrow[x] = 0;
				}
				gt_count[surfrow[x]]++;
			}
		}
		gt_counts.push_back(gt_count);

		// Populate histogram
		bin_maps.push_back(MatI());
		MatI& bin_map = bin_maps.back();
		bin_map.Resize(sz.y, sz.x);
		for (int y = 0; y < sz.y; y++) {
			int* surfrow = gt_surfs[y];
			int* binrow = bin_map[y];
			PixelRGB<byte>* imagerow = kf->image.rgb[y];
			for (int x = 0; x < sz.x; x++) {
				binrow[x] = layout.GetBinIndex(imagerow[x]);
				map_hist[surfrow[x]].IncrementBin(binrow[x]);
			}
		}
	}

	// Compute histogram sums
	int hist_sums[3];
	for (int i = 0; i < 3; i++) {
		hist_sums[i] = accumulate_all(map_hist[i].bins(), 0);
		DREPORT(i, hist_sums[i])
	}

	// Precompute the predictions for each bin
	int nbins = layout.GetBinCount();
	int cached_predictions[nbins];
	int prediction_share[] = {0,0,0};
	for (int i = 0; i < nbins; i++) {
		double maxp = -1.0;
		for (int j = 0; j < 3; j++) {
			double pj = 1.0*map_hist[j].bin(i) / hist_sums[j];
			if (pj > maxp) {
				maxp = pj;
				cached_predictions[i] = j;
				prediction_share[j]++;
			}
		}
	}
	DREPORT(iowrap(array_range(prediction_share, 3)));

	// The file pattern scheme
	format filepat("out/frame%03d_%s");

	// Output model predictions
	int ncorrect = 0, nevals = 0;
	COUNTED_FOREACH(int i, const proto::TruthedFrame& gt_frame, gt_map.frame()) {
		TITLE(i);

		// Pull out the frame
		const KeyFrame* kf = map.KeyFrameById(gt_frame.id());
		if (kf == NULL) {
			DLOG << "Warning: key frame " << gt_frame.id() << " not found in the map";
			continue;
		}
		ImageRef sz = kf->image.sz();

		// Get ground truth
		const MatI& gt_surfs = gt_surf_maps[i];
		const MatI& bin_map = bin_maps[i];

		// Compute the prior for this image from ground truth
		Vec3 gt_prior = gt_counts[i] / (1.0*sz.x*sz.y);
		DREPORT(gt_prior);

		// Compute predictions
		MatI prediction(sz.y, sz.x);
		MatI errors(sz.y, sz.x);

		int bin_predicts[nbins];
		for (int j = 0; j < nbins; j++) {
			double maxp = -1.0;
			for (int k = 0; k < 3; k++) {
				double p = 1.0*map_hist[k].bin(j);// * gt_prior[k] / hist_sums[k];
				//double p = map_hist[k].bin(j);
				if (p > maxp) {
					maxp = p;
					bin_predicts[j] = k;
				}
			}
		}

		/*MatF classcond[3];
		for (int j = 0; j < 3; j++) {
			classcond[j].Resize(sz.y, sz.x);  // class-conditional probabilities
		}*/
		for (int y = 0; y < sz.y; y++) {
			int* predrow = prediction[y];
			const int* gtrow = gt_surfs[y];
			const int* binrow = bin_map[y];
			PixelRGB<byte>* imagerow = kf->image.rgb[y];
			for (int x = 0; x < sz.x; x++) {
				int bin = binrow[x];
				predrow[x] = bin_predicts[bin];
				/*double maxp = -1;
				for (int j = 0; j < 3; j++) {
					classcond[j][y][x] = p;
				}*/
				if (predrow[x] == gtrow[x]) {
					ncorrect++;
				}
				nevals++;
			}
		}

		// Draw the original
		WriteImage(str(filepat % gt_frame.id() % "orig.png"), kf->image.rgb);

		// Draw the class-conditional probability maps
		/*for (int j = 0; j < 3; j++) {
			string ccfile = str(filepat % gt_frame.id() % str(format("cc%d.png")%j));
			WriteMatrixImageRescaled(ccfile, classcond[j]);
		}
		*/

		// Draw predicted orientations
		ImageRGB<byte> prediction_canvas;
		ImageCopy(kf->image.rgb, prediction_canvas);
		DrawOrientations(prediction, prediction_canvas, 0.5);
		WriteImage(str(filepat % gt_frame.id() % "prediction.png"), prediction_canvas);

		// Draw ground truth orientations
		ImageRGB<byte> gt_canvas;
		ImageCopy(kf->image.rgb, gt_canvas);
		DrawOrientations(gt_surfs, gt_canvas, 0.5);
		WriteImage(str(filepat % gt_frame.id() % "gt.png"), gt_canvas);
	}

	double accuracy = 100.0*ncorrect / nevals;
	DLOG << "Overall Accuracy: " << accuracy << " (" << ncorrect << " of " << nevals << ")";

	/*TITLED("Global Histograms") {
		for (int i = 0; i < 3; i++) {
			TITLED(format("Orient %d:")%i) {
				map_hist[i].RenderText();
			}
		}
	}*/

	return 0;
}
