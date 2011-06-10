#include <unistd.h>

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
#include "point_cloud_payoffs.h"
#include "floorplan_renderer.h"
#include "building_features.h"

#include "canvas.h"
#include "safe_stream.h"

#include "svm_helpers.h"
#include "svm_light_wrappers.h"

#include "format_utils.tpp"
#include "io_utils.tpp"
#include "counted_foreach.tpp"

using namespace std;
using namespace indoor_context;
using boost::format;

lazyvar<int> gvBalanceC("Bootstrap.BalanceC");
lazyvar<int> gvNumIterations("Bootstrap.NumIterations");
lazyvar<int> gvInitSampleSize("Bootstrap.InitSampleSize");
lazyvar<int> gvBalanceInitSamples("Bootstrap.BalanceInitSamples");

lazyvar<int> gvIncSampleSize("Bootstrap.IncSampleSize");
lazyvar<int> gvBalanceIncSamples("Bootstrap.BalanceIncSamples");

lazyvar<int> gvFeatureStride("Bootstrap.FeatureStride");

lazyvar<int> gvDrawSolutionsPeriod("Bootstrap.DrawSolutionsPeriod");
lazyvar<int> gvDrawActivePeriod("Bootstrap.DrawActivePeriod");
lazyvar<int> gvDrawPayoffsPeriod("Bootstrap.DrawPayoffsPeriod");
lazyvar<int> gvDrawObjectivesPeriod("Bootstrap.DrawObjectivesPeriod");

bool ThisIter(int iter, int period) {
	return period != 0 && (iter%period) == 0;
}

int main(int argc, char **argv) {
	InitVars();

	if (argc < 3 || argc > 4) {
		cout << "Usage: generate_svm_problem SEQUENCE FRAME_IDS [FEATURE_SET]" << endl;
		return 0;
	}

	// Parse arguments
	string sequence_name = argv[1];
	fs::path sequences_dir("sequences");
	fs::path rel_map_path("ground_truth/truthed_map.pro");
	fs::path map_file = sequences_dir/sequence_name/rel_map_path;
	vector<int> ids = ParseMultiRange<int>(argv[2]);
	string feature_set = argc >= 4 ? argv[3] : "default";

	// Load the sequence
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(map_file.string(), gt_map);
	double zceil = gt_map.floorplan().zceil();
	double zfloor = gt_map.floorplan().zfloor();
	const int stride = *gvFeatureStride;

	// Setup frames
	ptr_vector<FrameCase> frame_cases;
	
	// Generate features
	ptr_vector<PixelCase> cases;
	vector<PixelCase*> cases_by_label[3];
	vector<SVMLightCase*> all_cases;
	{
		// Put inside scope so it will be deleted afterwards
		BuildingFeatures ftrgen(feature_set);
		COUNTED_FOREACH(int index, int id, ids) {
			DLOG << "Generating features for frame " << id;

			// Load image
			KeyFrame& frame = *map.KeyFrameByIdOrDie(id);
			frame.LoadImage();
			frame.image.BuildMono();

			// Configure frame-level info
			FrameCase* fcase = new FrameCase;
			fcase->frame_id = id;
			fcase->image = &frame.image;
			fcase->geometry.Configure(frame.image.pc(), zfloor, zceil);
			fcase->ground_truth.Compute(gt_map.floorplan(), frame.image.pc());
			frame_cases.push_back(fcase);

			// Generate pixel features
			ftrgen.Compute(frame.image, &fcase->ground_truth.orientations());
			CHECK_SAME_SIZE(fcase->ground_truth.orientations(), *ftrgen.features[0]);

			// Copy to case vector
			for (int y = 0; y < frame.image.ny(); y += stride) {
				const int* gtrow = fcase->ground_truth.orientations()[y];
				for (int x = 0; x < frame.image.nx(); x += stride) {
					PixelCase* c = new PixelCase;
					c->feature.Resize(ftrgen.features.size());
					c->frame_id = id;
					c->frame_index = index;
					c->label = gtrow[x]+1;  // SVM-light is 1-based
					c->location = makeVector(x,y);
					COUNTED_FOREACH(int j, const MatF* ftrs, ftrgen.features) {
						c->feature[j] = (*ftrs)[y][x];
					}
					cases.push_back(c);
					all_cases.push_back(c);
					cases_by_label[c->label-1].push_back(c);  // SVM-light is 1-based
					fcase->pixels.push_back(c);
				}
			}
		}

		ofstream ftr_text("out/features.txt");
		for (int i = 0; i < ftrgen.features.size(); i++) {
			ftr_text << format("%03d: %s\n") % i % ftrgen.feature_strings[i];
		}
	}

	//
	// Begin bootstrap learning
	//

	// Generate a problem file containing all cases
	string all_prob_name = "allcases";
	TIMED("Writing complete case file")
		WriteSVMProblem(all_cases, all_prob_name);

	// Sample uniformly (but ballance the class counts)
	vector<SVMLightCase*> current_cases;
	if (*gvBalanceInitSamples) {
		CHECK_EQ(*gvInitSampleSize % 3, 0) << EXPR_STR(*gvInitSampleSize);
		int samples_per_class = *gvInitSampleSize / 3;
		for (int i = 0; i < 3; i++) {
			const vector<PixelCase*>& cur = cases_by_label[i];
			for (int j = 0; j < samples_per_class; j++) {
				current_cases.push_back(cur[rand() % cur.size()]);
			}
		}
	} else {
		for (int i = 0; i < *gvInitSampleSize; i++) {
			current_cases.push_back(&cases[rand() % cases.size()]);
		}
	}

	// Iterate bootstrap learning
	vector<int> classes(cases.size());  // Note that the values in here are 1-based due to SVM-light
	MatF responses(cases.size(), 3);
	for (int iter = 0; iter < *gvNumIterations; iter++) {
		TITLE("\nIteration "<<iter);

		// Count how many in each class
		int class_counts[] = {0,0,0};
		BOOST_FOREACH(const SVMLightCase* c, current_cases) {
			int label = c->label-1;   // SVM-light is 1-based
			CHECK_INTERVAL(label, 0, 2);
			class_counts[label]++;
		}
		DLOG << "Class counts: " << class_counts[0] << " " << class_counts[1] << " " << class_counts[2];

		TIMED_SECTION("Training new SVM") {
			// Generate the problem file and train SVM
			string prob_name = fmt("iter%03d", iter);
			WriteSVMProblem(current_cases, prob_name);
			TrainMultiClassSVM(prob_name);

			// Evaluate the SVM on the entire test set
			EvaluateMultiClassSVM(all_prob_name, prob_name);
			ReadMultiClassSVMResponses(all_prob_name, classes, responses);
		}

		// Copy the classification output to image coords
		TIMED("Assemble DP objective") {
			// Initialize matrices
			BOOST_FOREACH(FrameCase& fc, frame_cases) {
				for (int i = 0; i < 3; i++) {
					fc.responses[i].Resize(ceili(1.*fc.image->ny()/stride),
																 ceili(1.*fc.image->nx()/stride),
																 0);
				}
			}

			// Copy to reduced grid
			for (int i = 0; i < cases.size(); i++) {
				const PixelCase& c = cases[i];
				CHECK_INDEX(c.frame_index, frame_cases);
				FrameCase& fc = frame_cases[c.frame_index];
				Vec2I pos = makeVector(c.location[0]/stride,
															 c.location[1]/stride);
				for (int j = 0; j < 3; j++) {
					CHECK_POS(pos, fc.responses[j]) << EXPR_STR(c.location), EXPR_STR(stride);
					fc.responses[j][ pos[1] ][ pos[0] ] = responses[i][j];
				}
			}

			// Construct the DPObjective
			BOOST_FOREACH(FrameCase& fc, frame_cases) {
				fc.obj.Resize(fc.geometry.grid_size);
				for (int y = 0; y < fc.geometry.ny(); y++) {
					for (int x = 0; x < fc.geometry.nx(); x++) {
						Vec2 image_pt = project(fc.geometry.GridToImage(makeVector(x,y)));
						int im_x = roundi(image_pt[0] / stride);
						int im_y = roundi(image_pt[1] / stride);
						if (im_x >= 0 && im_x < fc.responses[0].Cols() &&
								im_y >= 0 && im_y < fc.responses[0].Rows()) {
							for (int j = 0; j < 3; j++) {
								fc.obj.pixel_scores[j][y][x] = fc.responses[j][im_y][im_x];
							}
						}
					}
				}
			}
		}

		// Run DP and sample cases for next training
		double sum_acc = 0;
		vector<PixelCase*> mistakes;
		vector<PixelCase*> mistakes_by_label[3];
		TIMED_SECTION("Evaluate current classifier")
		BOOST_FOREACH(FrameCase& fc, frame_cases) {
			TITLE("Frame " << fc.frame_id);

			format filepat("out/iter%03d_frame%03d_%s.png");
			filepat.bind_arg(1, iter);
			filepat.bind_arg(2, fc.frame_id);

			// Reconstruct
			ManhattanDPReconstructor recon;
			recon.Compute(*fc.image, fc.geometry, fc.obj);
			DREPORT(recon.dp.solution.score);

			// Find mistakes made by the DP
			int jj = 0;
			for (int y = 0; y < fc.image->ny(); y += stride) {
				for (int x = 0; x < fc.image->nx(); x += stride) {
					PixelCase& c = fc.pixels[jj];
					CHECK_EQ(c.location, makeVector(x,y));
					int gt_label = fc.ground_truth.orientations()[y][x];
					int soln_label = recon.dp.soln_orients[y][x];
					if (gt_label != soln_label) {
						mistakes.push_back(&c);
						mistakes_by_label[c.label-1].push_back(&c); // SVM-struct is 1-based
					}
					jj++;
				}
			}

			// Vizualize
			TIMED("Visualize") {
				if (ThisIter(iter, *gvDrawSolutionsPeriod)) {
					recon.OutputSolution(str(filepat % "dp"));
				}

				if (ThisIter(iter, *gvDrawActivePeriod)) {
					FileCanvas active_canvas(str(filepat % "active"), fc.image->rgb);
					BOOST_FOREACH(const SVMLightCase* c, current_cases) {
						const PixelCase* p = reinterpret_cast<const PixelCase*>(c);
						active_canvas.DrawDot(p->location, 3.5, Colors::red());
					}
				}
				
				// Visualize payoffs
				if (ThisIter(iter, *gvDrawPayoffsPeriod)) {
					for (int i = 0; i < 2; i++) {
						recon.OutputPayoffsViz(i, str(filepat % fmt("payoffs%d",i)));
					}
				}

				if (ThisIter(iter, *gvDrawObjectivesPeriod)) {
					// Visualize objective
					for (int i = 0; i < 3; i++) {
						WriteMatrixImageRecentred(str(filepat%fmt("obj%d",i)), fc.obj.pixel_scores[i]);
					}
				}
			}

			sum_acc += recon.ReportAccuracy(fc.ground_truth);
		}

		// Report overall accuracy
		DLOG << str(format("AVERAGE ACCURACY: %|40t|%.1f%%") % (100. * sum_acc / frame_cases.size()));

		// Sample some failure cases for the next round
		if (*gvBalanceIncSamples) {
			CHECK_EQ(*gvIncSampleSize%3, 0) << EXPR_STR(*gvIncSampleSize);
			int samples_per_label = *gvIncSampleSize / 3;
			for (int i = 0; i < 3; i++) {
				// If there are no mistakes with this label then select randomly from all cases
				const vector<PixelCase*>& cur =
					mistakes_by_label[i].empty() ? cases_by_label[i] : mistakes_by_label[i];
				if (mistakes_by_label[i].empty()) {
					DLOG << "No mistakes with label "<<i<<". Sampling uniformly from training set.";
				}
				for (int j = 0; j < samples_per_label; j++) {
					current_cases.push_back(cur[rand() % cur.size()]);
				}
			}
		} else {
			for (int i = 0; i < *gvIncSampleSize; i++) {
				current_cases.push_back(mistakes[rand() % mistakes.size()]);
			}
		}
	}

	return 0;
}
