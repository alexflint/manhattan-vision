#include <set>
#include <algorithm>

#include <boost/format.hpp>

#include "building_features.h"
#include "common_types.h"

#include "io_utils.tpp"
#include "image_utils.tpp"

namespace indoor_context {

	lazyvar<string> gvDefaultFeatureSet("BuildingFeatures.DefaultSet");
	lazyvar<int> gvGaborScales("BuildingFeatures.GaborScales");
	lazyvar<int> gvGaborOrientations("BuildingFeatures.GaborOrientations");

	BuildingFeatures::BuildingFeatures() {
	}

	BuildingFeatures::BuildingFeatures(const string& config) {
		Configure(config);
	}

	void BuildingFeatures::Configure(const string& config) {
		components.clear();

		string feature_set;
		if (config == "default") {
			feature_set = *gvDefaultFeatureSet;
		} else {
			feature_set = config;
		}

		// Build the set of all available components
		const char* known_comps[5] = {"rgb", "hsv", "gabor", "sweeps", "gt"};
		set<string> available;
		for (int i = 0; i < 5; i++) {
			available.insert(known_comps[i]);
		}
		
		// Build a list of components to be included when "all" is specified
		const char* normal_comps[4] = {"rgb", "hsv", "gabor", "sweeps"};
		set<string> normal;
		for (int i = 0; i < 4; i++) {
			normal.insert(normal_comps[i]);
		}

		// Parse the config string
		bool use_all = false;
		set<string> include, exclude;
		int end, start = 0;
		do {
			end = feature_set.find(',', start);
			int len = (end == string::npos) ? string::npos : end-start;
			string token = feature_set.substr(start, len);

			if (!token.empty() && token[0] == '-') {
				exclude.insert(token.substr(1));
			} else if (token == "all") {
				use_all = true;
			} else {
				include.insert(token);
			}
			start = end+1;
		} while (end != string::npos);

		// Resolve components
		if (use_all) {
			set_difference(normal.begin(), normal.end(),
										 exclude.begin(), exclude.end(),
										 inserter(components, components.begin()));
		}

		// Do this even when 'all' is specified in order to allow strings like "all gt"
		set_intersection(available.begin(), available.end(),
										 include.begin(), include.end(),
										 inserter(components, components.begin()));
		DLOG << "Using features: " << iowrap(components);

		// Find any unrecognised components
		set<string> unrecognised;
		set_difference(include.begin(), include.end(),
									 available.begin(), available.end(),
									 inserter(unrecognised, unrecognised.begin()));
		set_difference(exclude.begin(), exclude.end(),
									 available.begin(), available.end(),
									 inserter(unrecognised, unrecognised.begin()));
		if (!unrecognised.empty()) {
			DLOG << "Error, unrecognised components: " << iowrap(unrecognised);
		}

		// Configure the gabor filters (whether or not gabor filtering is enabled)
		gabor.Configure(*gvGaborScales, *gvGaborOrientations);
	}

	bool BuildingFeatures::IsActive(const string& component) {
		return components.find(component) != components.end();
	}

	void BuildingFeatures::Compute(const PosedImage& image, const MatI* gt_orients) {
		input = &image;
		features.clear();  // these are only pointers so no harm in clearing
		feature_strings.clear();

		// Convert to matrix
		MatF m;
		ImageToMatrix(image, m);
		
		//
		// RGB features
		//
		if (IsActive("rgb")) {
			rgb_features.resize(3);
			for (int i = 0; i < 3; i++) {
				rgb_features[i].Resize(image.ny(), image.nx());
				features.push_back(&rgb_features[i]);
			}
			feature_strings.push_back("R channel");
			feature_strings.push_back("G channel");
			feature_strings.push_back("B channel");

			for (int y = 0; y < image.ny(); y++) {
				float* rrow = rgb_features[0][y];
				float* grow = rgb_features[1][y];
				float* brow = rgb_features[2][y];
				const PixelRGB<byte>* imrow = image.rgb[y];
				for (int x = 0; x < image.nx(); x++) {
					rrow[x] = imrow[x].r;
					grow[x] = imrow[x].g;
					brow[x] = imrow[x].b;
				}
			}
		}

		//
		// HSV features
		//
		if (IsActive("hsv")) {
			hsv_features.resize(3);
			for (int i = 0; i < 3; i++) {
				hsv_features[i].Resize(image.ny(), image.nx());
				features.push_back(&hsv_features[i]);
			}
			feature_strings.push_back("H channel");
			feature_strings.push_back("S channel");
			feature_strings.push_back("V channel");

			byte h, s, v;
			for (int y = 0; y < image.ny(); y++) {
				float* hrow = hsv_features[0][y];
				float* srow = hsv_features[1][y];
				float* vrow = hsv_features[2][y];
				const PixelRGB<byte>* imrow = image.rgb[y];
				for (int x = 0; x < image.nx(); x++) {
					VW::ImageConversions::RGB2HSV(imrow[x].r, imrow[x].g, imrow[x].b, h, s, v);
					hrow[x] = h;
					srow[x] = s;
					vrow[x] = v;
				}
			}
		}
		CHECK_EQ(features.size(), feature_strings.size());

		//
		// Gabor filter responses
		//
		if (IsActive("gabor")) {
			int div = 1<<gabor.num_scales;
			CHECK_EQ(image.nx() % div, 0) << "Image dimensions must be exactly divisible by 2^num_scales";
			CHECK_EQ(image.ny() % div, 0) << "Image dimensions must be exactly divisible by 2^num_scales";

			gabor.Run(m);
			gabor_features.resize(gabor.responses.size());
			for (int i = 0; i < gabor.responses.size(); i++) {
				const MatF& response = *gabor.responses[i];
				gabor_features[i].Resize(image.ny(), image.nx());
				Upsample(response, gabor_features[i], image.ny()/response.Rows());
				features.push_back(&gabor_features[i]);

				int orient_index = i/gabor.num_scales;
				float orient = orient_index*180.0/gabor.num_orients;  // conversion to degrees from MakeGaborFilters in filters.cpp
				int scale = 1<<(i%gabor.num_orients);
				feature_strings.push_back(str(format("Gabor orient=%.1f scale=%dx") % orient % scale));
			}
		}
		CHECK_EQ(features.size(), feature_strings.size());

		//
		// Line sweeps
		//
		if (IsActive("sweeps")) {
			line_detector.Compute(image);
			line_sweeper.Compute(image, line_detector.detections);
			sweep_features.resize(3);
			for (int i = 0; i < 3; i++) {
				sweep_features[i].Resize(image.ny(), image.nx());
				features.push_back(&sweep_features[i]);
				feature_strings.push_back(str(format("Line sweeps for orient %d")%i));
			}
			for (int y = 0; y < image.ny(); y++) {
				int* sweeprow = line_sweeper.orient_map[y];
				for (int i = 0; i < 3; i++) {
					float* outrow = sweep_features[i][y];
					for (int x = 0; x < image.nx(); x++) {
						outrow[x] = (sweeprow[x]==i) ? 1.0 : 0.0;
					}
				}
			}
		}
		CHECK_EQ(features.size(), feature_strings.size());

		//
		// Ground truth
		//
		if (IsActive("gt")) {
			DLOG << "Warning: adding GROUND TRUTH to features";
			CHECK_NOT_NULL(gt_orients)
				<< "'gt' was requested in the feature set but no ground truth provided to BuildingFeatures::Configure()";
			CHECK_SAME_SIZE(*gt_orients, image);
			gt_features.resize(3);
			for (int i = 0; i < 3; i++) {
				gt_features[i].Resize(image.ny(), image.nx());
				features.push_back(&gt_features[i]);
				feature_strings.push_back(str(format("Ground truth for orient %d")%i));
			}
			for (int y = 0; y < image.ny(); y++) {
				const int* gtrow = (*gt_orients)[y];
				for (int i = 0; i < 3; i++) {
					float* outrow = gt_features[i][y];
					for (int x = 0; x < image.nx(); x++) {
						outrow[x] = (gtrow[x]==i) ? 1.0 : 0.0;
					}
				}
			}
		}
		CHECK_EQ(features.size(), feature_strings.size());
	}
}
