#include <mex.h>

#include <boost/ptr_container/ptr_map.hpp>
#include <boost/filesystem.hpp>

#include "manhattan_inference.h"

#include "common_types.h"
#include "manhattan_dp.h"
#include "camera.h"
#include "timer.h"
#include "clipping.h"
#include "bld_helpers.h"
#include "safe_stream.h"

#include "io_utils.tpp"
#include "table.tpp"


namespace indoor_context {
	using namespace toon;
	using boost::ptr_map;

	lazyvar<double> gvPerPixelLoss("ManhattanInference.PerPixelLoss");

	void ManhattanInference::Prepare(const PosedImage& image,
																	 const proto::FloorPlan& floorplan,
																	 shared_array<FeatureVec> features) {
		Mat3 floorToCeil = GetFloorCeilHomology(image.pc(), floorplan);
		Prepare(image, floorToCeil, features);
		ComputeGroundTruth(floorplan);
	}

	void ManhattanInference::Prepare(const PosedImage& image,
																	 const Mat3& floorToCeil,
																	 shared_array<FeatureVec> features) {
		input_image = &image;
		input_floorToCeil = floorToCeil;
		input_features = features;
	}

	void ManhattanInference::ComputeGroundTruth(const proto::FloorPlan& floorplan) {
		CHECK(input_image) << "Prepare() must be called first";
		GetTrueOrients(floorplan, input_image->pc(), gt_labels);
	}

	void ManhattanInference::ComputeReconstruction(const Vector<>& weights) {
		ComputeScoreFunc(weights, NULL);
		reconstructor.Compute(*input_image, input_floorToCeil, score_func);
	}

	void ManhattanInference::ComputeMostViolated(const Vector<>& weights) {
		CHECK_GT(gt_labels.Rows(), 0) <<
			"ComputeMostViolated called with only one argument "
			"but ground truth was not provided to Prepare";
		ComputeMostViolated(weights, gt_labels);
	}

	void ManhattanInference::ComputeMostViolated(const Vector<>& weights,
																							 const MatI& ref_labels) {
		ComputeScoreFunc(weights, &ref_labels);
		reconstructor.Compute(*input_image, input_floorToCeil, score_func);
	}

	void ManhattanInference::ComputePsi(const MatI& labels,
																			ManhattanInference::PsiVec& psi) {
		CHECK(input_image) << "Prepare() must be called first";
		//CHECK_EQ(labels.Rows()*labels.Cols(), feature_gen.feature_count);

		int i = 0;
		FeatureVec ftr_sums[3];
		for (int y = 0; y < feature_gen.dims[1]; y++) {
			const int* labelrow = labels[y];
			for (int x = 0; x < feature_gen.dims[0]; x++) {
				ftr_sums[ labelrow[x] ] += input_features[i++];
			}
		}

		static const int kLen = LineSweepFeatureGenerator::kFeatureLength;
		for (int i = 0; i < 3; i++) {
			psi.slice(kLen*i, kLen) = ftr_sums[i];
		}
	}

	void ManhattanInference::ComputeScoreFunc(const toon::Vector<>& weights,
																						const MatI* ref_labels) {
		CHECK(input_image) << "Prepare() must be called first";
		const double kLoss = *gvPerPixelLoss;
		if (ref_labels != NULL) {
			CHECK_EQ(ref_labels->Rows(), feature_gen.dims[1]);
			CHECK_EQ(ref_labels->Cols(), feature_gen.dims[0]);
		}

		// Compute the score function
		score_func.Resize(feature_gen.dims[0], feature_gen.dims[1]);
		Vector<kFeatureLength> classifiers[] = {
				weights.slice(0, kFeatureLength),
				weights.slice(kFeatureLength, kFeatureLength),
				weights.slice(kFeatureLength*2, kFeatureLength)
		};

		int i = 0;
		for (int y = 0; y < feature_gen.dims[1]; y++) {
			float* scorerows[] = {
					score_func.pixel_scores[0][y],
					score_func.pixel_scores[1][y],
					score_func.pixel_scores[2][y]
			};
			const int* refrow = ref_labels ? (*ref_labels)[y] : NULL;
			for (int x = 0; x < feature_gen.dims[0]; x++) {
				const Vector<kFeatureLength>& ftr = input_features[i++];
				for (int i = 0; i < 3; i++) {
					scorerows[i][x] = classifiers[i] * ftr;
					if (ref_labels != NULL && refrow[x] != i) {
						// When finding the most violated constraint, increase the score
						// for all the labels _other_ than the ground truth
						scorerows[i][x] += kLoss;
					}
				}
			}
		}
	}




	lazyvar<string> gvDataDir("Map.DataDir");

	//static
	FrameStore& FrameStore::instance() {
		static FrameStore store;
		return store;
	}
	
	// static
	pair<string,int> FrameStore::ParseCaseName(const string& case_name) {
		int colon_pos = case_name.find(':');
		CHECK(colon_pos != string::npos) << "Malformed case name: " << case_name;
		string sequence_name = case_name.substr(0, colon_pos);
		int frame_index = lexical_cast<int>(case_name.substr(colon_pos+1));
		return make_pair(sequence_name, frame_index);
	}

	ManhattanInference& FrameStore::Get(const string& case_name) {
		CHECK(cases.find(case_name) != cases.end())
			<< "case name: " << case_name;
		return cases[case_name];
	}

	ManhattanInference& FrameStore::GetOrInit(const string& case_name) {
		return cases[case_name];
	}

	/* Frame& FrameStore::GetFrame(const string& case_name) {
		pair<string,int> spec = ParseCaseName(case_name);
		KeyFrame& kf = *GetMap(spec.first).KeyFrameByIdOrDie(spec.second);
		kf.LoadImage(); // leave image loaded for now
		return kf;
		}*/

	Map& FrameStore::GetMap(const string& sequence_name) {
		EnsureMapLoaded(sequence_name);
		return maps[sequence_name];
	}

  const proto::FloorPlan& FrameStore::GetFloorPlan(const string& sequence_name) {
		EnsureMapLoaded(sequence_name);
		return gt_maps[sequence_name].floorplan();
	}

  void FrameStore::EnsureMapLoaded(const string& sequence_name) {
		if (maps.find(sequence_name) == maps.end()) {
			// Get the path to the map
			fs::path sequence_dir = fs::path(*gvDataDir) / sequence_name;;
			fs::path tru_map_file = sequence_dir / "ground_truth/truthed_map.pro";
			CHECK_PRED(fs::is_directory, sequence_dir);
			CHECK_PRED(fs::is_regular_file, tru_map_file);

			// Load the map
			DLOG << "Loading map: " << tru_map_file;
			maps[sequence_name].LoadWithGroundTruth(tru_map_file.string(),
																							gt_maps[sequence_name]);
		}
	}


	void FrameStore::Clear() {
		maps.clear();
		gt_maps.clear();
		cases.clear();
	}
}
