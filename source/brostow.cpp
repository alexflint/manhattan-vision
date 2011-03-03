#include <iostream>

#include <stdio.h>
#include <boost/filesystem.hpp>

#include <VW/Image/imagecopy.tpp>

#include "entrypoint_types.h"
#include "map.h"
#include "bld_helpers.h"
#include "safe_stream.h"
#include "timer.h"
#include "manhattan_ground_truth.h"
#include "svm_light_wrappers.h"
#include "svm_helpers.h"

#include "integral_image.tpp"
#include "counted_foreach.tpp"
#include "format_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"

lazyvar<double> gvWindowSize("Brostow.WindowSize");
lazyvar<double> gvMarginPenalty("Brostow.MarginPenalty");
lazyvar<double> gvNumTrainingPixels("Brostow.NumTrainingPixels");
lazyvar<double> gvTestPixelStride("Brostow.TestPixelStride");

static const string kModelName = "brostow_mc";
static const int kNumClasses = 3;

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc < 3) {
		DLOG << "Usage: brostow SEQUENCE FRAMES [RESULTS_DIR|train]";
		DLOG << "           if specified, results will be appended to RESULTS_DIR/out";
		exit(-1);
	}

	// Read parameters
	string sequence = argv[1];
	vector<int> frame_ids = ParseMultiRange<int>(argv[2]);

	bool train = false;
	fs::path results_dir;
	if (argc < 4) {
		results_dir = fs::initial_path();
	} else if (string(argv[3]) == "train") {
		train = true;
		DLOG << "Training";
	} else {
		results_dir = argv[3];
		CHECK_PRED1(fs::exists, results_dir);
	}

	// Open file for appending statistics to
	fs::path stats_file = results_dir / fmt("performance_brostow_%s.csv", results_dir.filename());
	ofstream stats_out(stats_file.string().c_str(), ios::app);
	format stats_fmt("\"%s\",\"%s\",%d,%f\n");
	
	// Pattern for output filenames
	fs::path viz_dir = results_dir / "out";
	if (!fs::create_directory(viz_dir)) {
		DLOG << "Created output directory: " << viz_dir;
	}
	format filepat("%s/%s_frame%03d_%s.%s");
	filepat.bind_arg(1, viz_dir.string());
	filepat.bind_arg(2, sequence);

	// Write parameters and initialize the performance CSV
	if (!train) {
		// Write the parameters
		fs::path params_file = results_dir / fmt("parameters_%s.csv", results_dir.filename());
		ofstream params_out(params_file.string().c_str());
		params_out << "\"Brostow.WindowSize\"," << *gvWindowSize << endl;
		params_out.close();
	}

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	// Compute point-camera distances and average reprojection error
	vector<double> pt_dists(map.pts.size());
	fill_all(pt_dists, INFINITY);
	for (int i = 0; i < map.kfs.size(); i++) {
		Vec3 c = map.kfs[i].image.pc().world_centre();
		for (int j = 0; j < map.pts.size(); j++) {
			// Compute distance to this keyframe
			double dsqr = norm_sq(map.pts[j]-c);
			if (dsqr < pt_dists[j]) {
				pt_dists[j] = dsqr;
			}
		}
	}
	for (int i = 0; i < map.pts.size(); i++) {
		pt_dists[i] = sqrt(pt_dists[i]);
	}

	// Compute average reprojection errors
	vector<int> pt_observations(map.pts.size());
	vector<double> pt_residuals(map.pts.size());
	fill_all(pt_observations, 0);
	fill_all(pt_residuals, 0);
	BOOST_FOREACH(const KeyFrame& frame, map.kfs) {
		BOOST_FOREACH(const Measurement& msm, frame.measurements) {
			// Add reprojection error
			Vec2 reproj = project(frame.image.pc().WorldToIm(map.pts[msm.point_index]));
			pt_residuals[msm.point_index] += norm(reproj - msm.image_pos);
			pt_observations[msm.point_index]++;
		}
	}
	for (int i = 0; i < map.pts.size(); i++) {
		pt_residuals[i] /= pt_observations[i];
	}

	ptr_vector<VecF> feature_store;
	vector<VecF*> features;
	vector<int> labels;
	//int npos = 0, nneg = 0;  // number of positive/negative training examples

	// Compute features for each frame
	double sum_acc = 0;
	int num_frames = 0;
	BOOST_FOREACH (int frame_id, frame_ids) {
		TITLE("Processing frame "<<frame_id);
		scoped_timer t(fmt("Process frame %d", frame_id));

		filepat.bind_arg(3, frame_id);
		filepat.bind_arg(5, "png");

		// Get the frame
		KeyFrame& frame = *map.KeyFrameById(frame_id);
		if (&frame == NULL) continue;
		frame.LoadImage();

		// Get the camera centre
		Vec3 camera_centre = frame.image.pc().world_centre();

		// Project points into this frame
		int outside = 0;
		MatI counts(frame.image.ny(), frame.image.nx(), 0);
		MatF ydiffs(frame.image.ny(), frame.image.nx(), 0);
		MatF dists(frame.image.ny(), frame.image.nx(), 0);
		MatF residuals(frame.image.ny(), frame.image.nx(), 0);
		BOOST_FOREACH(const Measurement& msm, frame.measurements) {
			Vec2I p = RoundVector(msm.image_pos);
			if (frame.image.contains(asIR(p))) {
				CHECK_INDEX(msm.point_index, map.pts);
				CHECK_INDEX(msm.point_index, pt_dists);

				const Vec3& v = map.pts[msm.point_index];
				counts[ p[1] ][ p[0] ]++;
				ydiffs[ p[1] ][ p[0] ] += v[2] - camera_centre[2];  // in our coords, axis 2 is "up"
				dists[ p[1] ][ p[0] ] += pt_dists[msm.point_index];
				residuals[ p[1] ][ p[0] ] += pt_residuals[msm.point_index];
			} else {
				outside++;
			}
		}
		if (outside > 0) {
			DLOG << "Warning, there were "<<outside<<" points measured outside the image";
		}

		// Build integral images
		IntegralImage<int> intg_counts(counts);
		IntegralImage<float> intg_ydiffs(ydiffs);
		IntegralImage<float> intg_dists(dists);
		IntegralImage<float> intg_residuals(residuals);

		// Compute per-pixel features
		MatF density_ftr(frame.image.ny(), frame.image.nx());
		MatF dy_ftr(frame.image.ny(), frame.image.nx());
		MatF dist_ftr(frame.image.ny(), frame.image.nx());
		MatF residual_ftr(frame.image.ny(), frame.image.nx());

		// Compute pixel features
		Vec2 tl, br;
		const int kWin = *gvWindowSize;
		for (int y = 0; y < frame.image.ny(); y++) {
			tl[1] = Clamp<int>(y-kWin, 0, frame.image.ny()-1);
			br[1] = Clamp<int>(y+kWin, 0, frame.image.ny()-1);
			for (int x = 0; x < frame.image.nx(); x++) {
				tl[0] = Clamp<int>(x-kWin, 0, frame.image.nx()-1);
				br[0] = Clamp<int>(x+kWin, 0, frame.image.nx()-1);
				int count = intg_counts.Sum(tl, br);
				density_ftr[y][x] = 1.0 * count / ((br[0]-tl[0]+1)*(br[1]-tl[1]+1));
				if (count == 0) {
					dy_ftr[y][x] = 0;
					dist_ftr[y][x] = 0;
					residual_ftr[y][x] = 0;
				} else {
					dy_ftr[y][x] = intg_ydiffs.Sum(tl, br) / count;
					dist_ftr[y][x] = intg_dists.Sum(tl, br) / count;
					residual_ftr[y][x] = intg_residuals.Sum(tl, br) / count;
				}
			}
		}

		// Compute ground truth
		ManhattanGroundTruth gt(gt_map.floorplan(), frame.image.pc());

		// Compute stride
		int stride;
		if (train) {
			int ncases = *gvNumTrainingPixels;
			stride = ceili(sqrt(1.0*frame_ids.size()*frame.image.nx()*frame.image.ny()/ncases));
		} else {
			stride = *gvTestPixelStride;
		}
		CHECK_GT(stride, 0);

		// Generate features
		int num_gt_unknown = 0;
		int gt_counts[] = {0,0,0};
		for (int y = 0; y < frame.image.ny(); y += stride) {
			for (int x = 0; x < frame.image.nx(); x += stride) {
				VecF* ftr = new VecF(4);
				(*ftr)[0] = density_ftr[y][x];
				(*ftr)[1] = dy_ftr[y][x];
				(*ftr)[2] = dist_ftr[y][x];
				(*ftr)[3] = residual_ftr[y][x];
				feature_store.push_back(ftr);
				features.push_back(ftr);
				int orient = gt.orientations()[y][x];
				CHECK_INTERVAL(orient, -1, 3);
				if (orient == -1) {
					num_gt_unknown++;
					orient = 0;
				}
				gt_counts[orient]++;
				labels.push_back(orient + 1);  // +1 because svm_multiclass is 1-based
			}
		}
		if (num_gt_unknown > 0) {
			DLOG << "Warning: there were "<<num_gt_unknown<<" unkonwn (-1) orientations in ground truth, changing to label 0";
		}
		DLOG << "Ground truth label counts: " << gt_counts[0] << " " << gt_counts[1] << " " << gt_counts[2];

		// Write visualizations
		/*WriteMatrixImageRescaled(str(filepat % "density"), density_ftr);
		WriteMatrixImageRecentred(str(filepat % "dy"), dy_ftr);
		WriteMatrixImageRescaled(str(filepat % "dist"), dist_ftr);
		WriteMatrixImageRescaled(str(filepat % "residual"), residual_ftr);*/

		// Compute accuracy
		if (!train) {
			// Evaluate SVM
			string prob = fmt("brostow_frame%d", frame_id);
			DLOG << "Writing test cases";
			WriteSVMProblem(features, labels, prob);
			TITLED("Evaluating SVM")
				EvaluateMultiClassSVM(prob, kModelName);

			// Read responses
			int nright = 0, nwrong = 0;
			vector<int> classes(features.size());
			MatF responses(features.size(), kNumClasses);
			ReadMultiClassSVMResponses(prob, classes, responses);

			int out_counts[] = {0,0,0};
			for (int i = 0; i < features.size(); i++) {
				out_counts[classes[i]-1]++;
				if (classes[i] == labels[i]) {  // +1 because svm_multiclass if 1-based
					nright++;
				} else {
					nwrong++;
				}
			}

			DLOG << "Output class counts: " << out_counts[0] << " " << out_counts[1] << " " << out_counts[2];

			double pixel_acc = 100.0 * nright / features.size();
			sum_acc += pixel_acc;
			num_frames++;

			// Transform to image coords
			int i = 0;
			MatI orients(frame.image.ny(), frame.image.nx(), 0);
			for (int y = 0; y < frame.image.ny(); y++) {
				int* row = orients[y];
				for (int x = 0; x < frame.image.nx(); x++) {
					int i = (y/stride) * (frame.image.nx()/stride) + (x/stride);
					row[x] = classes[i] - 1; // -1 because svm_multiclass is 1-based
				}
 			}

			// Visualize
			ImageRGB<byte> canvas;
			ImageCopy(frame.image.rgb, canvas);
			DrawOrientations(orients, canvas, 0.35);
			WriteImage(str(filepat % "orients_brostow"), canvas);

			DLOG << format("Labelling accuracy: %|40t|%f%%\n") % pixel_acc;

			// Write results to CSV file
			time_t now = time(NULL);
			string timestamp = asctime(localtime(&now));
			stats_out << stats_fmt
				% timestamp.substr(0,timestamp.length()-1)
				% sequence
				% frame_id
				% pixel_acc;

			// Write results to individual file
			// this must come last because of call to bind_arg()
			filepat.bind_arg(5, "txt");
			sofstream info_out(str(filepat % "stats"));
			info_out << format("Labelling accuracy: %|40t|%f%%\n") % pixel_acc;

			// Clear features for next frame
			feature_store.clear();
			features.clear();
			labels.clear();
		}
	}

	// Ballance training
	vector<VecF*> ballanced_features;
	vector<int> ballanced_labels;
	vector<VecF*> features_by_label[3];
	for (int i = 0; i < features.size(); i++) {
		features_by_label[ labels[i]-1 ].push_back(features[i]);
	}
	int nmin = min(features_by_label[0].size(),
								 min(features_by_label[1].size(),
										 features_by_label[2].size()));
	COUNTED_FOREACH(int label, vector<VecF*>& v, array_range(features_by_label, 3)) {
		while (v.size() > nmin) {
			v.erase(v.begin() + (rand()%v.size()));
		}
		copy_all_into(v, ballanced_features);
		for (int i = 0; i < v.size(); i++) {
			ballanced_labels.push_back(label+1);  // +1 because svm_multiclass is 1-based
		}
	}

	// Train SVM?
	if (train) {
		CHECK_EQ(ballanced_features.size(), ballanced_labels.size());
		DLOG << "Writing " << ballanced_features.size() << " training cases";
		WriteSVMProblem(ballanced_features, ballanced_labels, kModelName);
		TITLED("Training SVM")
			TrainMultiClassSVM(kModelName, *gvMarginPenalty);
	} else {
		// Note that if one or more frames weren't found then num_frames
		// will not equal frame_ids.size()
		double av_acc = sum_acc / num_frames;  // these are already multipled by 100
		DLOG << format("AVERAGE PIXEL ACCURACY: %|40t|%.2f%%") % av_acc;
	}

	DLOG << "Done";
	return 0;
}
