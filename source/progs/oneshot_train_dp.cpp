#include <vector>
#include <list>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <VW/Image/imagecopy.h>

#include "entrypoint_types.h"
#include "common_types.h"
#include "manhattan_dp.h"
#include "map.h"
#include "timer.h"
#include "map.pb.h"
#include "bld_helpers.h"
#include "line_sweep_features.h"
#include "floorplan_renderer.h"
#include "building_features.h"
#include "svm_helpers.h"

#include "canvas.h"

#include "svm_light_wrappers.h"

#include "format_utils.tpp"
#include "io_utils.tpp"
#include "counted_foreach.tpp"

using namespace std;
using namespace indoor_context;
using boost::format;

lazyvar<string> gvTrainSequence("Oneshot.TrainSequence");
lazyvar<string> gvTrainFrames("Oneshot.TrainFrames");
lazyvar<int> gvNumTrainPixels("Oneshot.NumTrainPixels");

lazyvar<string> gvTestSequence("Oneshot.TestSequence");
lazyvar<string> gvTestFrames("Oneshot.TestFrames");
lazyvar<string> gvFeatureSet("Oneshot.FeatureSet");

lazyvar<int> gvVisualizeFeatures("Oneshot.VisualizeFeatures");

// Just for managing memory
FrameCase* GenerateFeatures(const KeyFrame& frame,
														const proto::TruthedMap& gt_map,
														int stride) {
	CHECK_GT(stride, 0);

	BuildingFeatures ftrgen(*gvFeatureSet);

	// Configure frame-level info
	FrameCase* fcase = new FrameCase;
	fcase->frame_id = frame.id;
	fcase->image = &frame.image;
	fcase->geometry.Configure(frame.image.pc(),
														gt_map.floorplan().zfloor(),
														gt_map.floorplan().zceil());
	fcase->ground_truth.Compute(gt_map.floorplan(),
															frame.image.pc());

	// Generate pixel features
	ftrgen.Compute(frame.image, &fcase->ground_truth.orientations());
	CHECK_SAME_SIZE(fcase->ground_truth.orientations(), *ftrgen.features[0]);

	// Copy to case vector
	for (int y = 0; y < frame.image.ny(); y += stride) {
		const int* gtrow = fcase->ground_truth.orientations()[y];
		for (int x = 0; x < frame.image.nx(); x += stride) {
			PixelCase* c = new PixelCase;
			c->feature.Resize(ftrgen.features.size());
			c->frame_id = frame.id;
			c->label = gtrow[x];
			c->location = makeVector(x,y);
			COUNTED_FOREACH(int j, const MatF* ftrs, ftrgen.features) {
				c->feature[j] = (*ftrs)[y][x];
			}
			fcase->pixels.push_back(c);
		}
	}				

	// Generate outputs
	if (*gvVisualizeFeatures) {
		for (int i = 0; i < ftrgen.features.size(); i++) {
			WriteMatrixImageRescaled(fmt("out/frame%03d_ftr%02d.png", frame.id, i),
															 *ftrgen.features[i]);
		}
	}

	ofstream ftr_text("out/features.txt");
	for (int i = 0; i < ftrgen.features.size(); i++) {
		ftr_text << format("%03d: %s\n") % i % ftrgen.feature_strings[i];
	}

	return fcase;
}

int main(int argc, char **argv) {
	InitVars();

	if (argc > 2) {
		cout << "Usage: oneshot_train_dp [skip_training]" << endl;
		return 0;
	}

	// Generate model names
	string model_names[3];
	for (int i = 0; i < 3; i++) {
		model_names[i] = fmt("orient%d_train", i);
	}

	// Only train a new model if no parameters given
	if (argc == 1) {
		// Load the sequence
		Map map;
		proto::TruthedMap gt_map;
		map.LoadWithGroundTruth(GetMapPath(*gvTrainSequence), gt_map);
		vector<int> train_ids = ParseMultiRange<int>(*gvTrainFrames);

		// Calculate feature spacing
		int stride = sqrt(640*480 * train_ids.size() / (*gvNumTrainPixels));
	
		// Compute features for training set
		ptr_vector<FrameCase> train_frames;
		vector<PixelCase*> train_pixels;
		BOOST_FOREACH(int id, train_ids) {
			TITLE("Generating features for training frame "<<id);

			// Load frame
			KeyFrame& frame = *map.KeyFrameByIdOrDie(id);
			frame.LoadImage();
			frame.image.BuildMono();

			// Generate features
			FrameCase* fcase = GenerateFeatures(frame, gt_map, stride);
			train_frames.push_back(fcase);
			BOOST_FOREACH(PixelCase& pcase, fcase->pixels) {
				train_pixels.push_back(&pcase);
			}
		}

		DREPORT(train_pixels.size());

		// Train one-vs-all classifiers
		for (int i = 0; i < 3; i++) {
			TITLE("\nTraining model " << i);

			// Generate features and labels
			vector<VecF*> features(train_pixels.size());
			vector<int> labels(train_pixels.size());
			int pve_count = 0, nve_count = 0;
			for (int j = 0; j < train_pixels.size(); j++) {
				features[j] = &train_pixels[j]->feature;
				labels[j] = (train_pixels[j]->label == i ? 1 : -1);
				if (labels[j] > 0) {
					pve_count++;
				} else {
					nve_count++;
				}
			}

			// Train SVM
			WriteSVMProblem(features, labels, model_names[i]);
			TrainSVM(model_names[i], 1.0*nve_count/pve_count);
		}
	}




	// Load the test sequence
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(*gvTestSequence), gt_map);
	
	// Evaluate each frame
	const int stride = *gvFeatureStride;
	vector<int> test_ids = ParseMultiRange<int>(*gvTestFrames);
	COUNTED_FOREACH(int index, int id, test_ids) {
		TITLE("Evaluating test frame "<<id);

		// Get the frame
		KeyFrame& frame = *map.KeyFrameByIdOrDie(id);
		frame.LoadImage();
		frame.image.BuildMono();

		// Generate features
		scoped_ptr<FrameCase> fcase;
		TIMED("Generate features")
			fcase.reset(GenerateFeatures(frame, gt_map, stride));

		// Compute the objective
		DPObjective obj(frame.image.size());
		for (int i = 0; i < 3; i++) {
			TITLE("Orientation "<<i);

			// Set up features and labels
			vector<VecF*> features(fcase->pixels.size());
			vector<int> labels(fcase->pixels.size());
			for (int j = 0; j < features.size(); j++) {
				features[j] = &fcase->pixels[j].feature;
				labels[j] = (fcase->pixels[j].label == i ? 1 : -1);
			}

			// Evaluate SVMs
			TIMED_SECTION("Evaluate SVM") {
				string prob = fmt("frame%03d_orient%d_test", id, i);
				WriteSVMProblem(features, labels, prob);
				EvaluateSVM(prob, model_names[i]);
			}

			// Read response
			vector<float> response_vec(fcase->pixels.size());
			ReadSVMResponses(prob, response_vec);

			// Compute SVM accuracy
			int svm_corr = 0;
			for (int j = 0; j < features.size(); j++) {
				if ((response_vec[j] < 0) == (labels[j] < 0)) {
					svm_corr++;
				}
			}
			float svm_acc = 100.0*svm_corr/features.size();
			DLOG << fmt("SVM accuracy: %.1f%%", svm_acc);

			// Transform to image coords
			MatF responses(frame.image.ny()/stride, frame.image.nx()/stride, 0);
			for (int j = 0; j < fcase->pixels.size(); j++) {
				const PixelCase& c = fcase->pixels[j];
				int x = c.location[0]/stride;
				int y = c.location[1]/stride;
				responses[y][x] = response_vec[j];
			}

			// Fill in the objective
			for (int y = 0; y < frame.image.ny(); y++) {
				float* row = obj.pixel_scores[i][y];
				for (int x = 0; x < frame.image.nx(); x++) {
					int xx = Clamp<int>(roundi(1.0*x/stride), 0, responses.Cols()-1);
					int yy = Clamp<int>(roundi(1.0*y/stride), 0, responses.Rows()-1);
					row[x] = responses[yy][xx];
				}
			}
		}

		// Reconstruct
		DPGeometry geom(frame.image.pc(), gt_map.floorplan().zfloor(), gt_map.floorplan().zceil());
		ManhattanDPReconstructor recon;
		recon.Compute(frame.image, geom, obj);

		// Report results
		double av_err = recon.GetMeanDepthError(fcase->ground_truth);
		double pixel_acc = recon.GetAccuracy(fcase->ground_truth);
		DLOG << fmt("Pixel-wise accuracy (of DP): %.1f%%", pixel_acc*100);
		DLOG << fmt("*** Mean depth error (of DP): %.1f%%", av_err*100);

		// Visualize
		format filepat("out/frame%03d_%s.png");
		filepat.bind_arg(1, id);
		recon.OutputSolution(str(filepat % "dp"));
		for (int i = 0; i < 3; i++) {
			TITLED(i) DREPORT(obj.pixel_scores[i].MinValue(), obj.pixel_scores[i].MaxValue());
			WriteMatrixImageRecentred(str(filepat % fmt("objective%d", i)), obj.pixel_scores[i]);
		}
	}

	return 0;
}
