#include "building_features.h"

#include <set>
#include <algorithm>

#include <boost/format.hpp>

#include "common_types.h"
#include "colors.h"

#include "io_utils.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "format_utils.tpp"

namespace indoor_context {

	lazyvar<string> gvDefaultFeatureSet("PhotometricFeatures.DefaultSet");
	lazyvar<int> gvGaborScales("PhotometricFeatures.GaborScales");
	lazyvar<int> gvGaborOrientations("PhotometricFeatures.GaborOrientations");

	lazyvar<int> gvFirstWindowForSweeps("PhotometricFeatures.FirstWindowForSweeps");
	lazyvar<int> gvNumScalesForSweeps("PhotometricFeatures.NumScalesForSweeps");

	void AccumulatedFeatures::Prepare(const MatF& input) {
		size = matrix_size(input);
		results.clear();
		integ.Compute(input);
	}

	void AccumulatedFeatures::Compute(const MatF& input, int radius, bool nbrs) {
		Prepare(input);
		Compute(radius, nbrs);
	}

	void AccumulatedFeatures::Compute(int r, bool nbrs) {
		CHECK_GT(integ.m_int.Rows(), 0)
			<< "AccumulatedFeatures::Prepare() must be called before Compute()";

		int n = nbrs ? 5 : 1;

		int k = results.size();
		results.resize(results.size()+n);

		// Acumulate features in local neighbourhoods
		Vec2I tl, br;
		results[k].Resize(size[1], size[0]);
		MatF& accumulated = results[k];
		for (int y = 0; y < size[1]; y++) {
			tl[1] = Clamp<int>(y-r, 0, size[1]-1);
			br[1] = Clamp<int>(y+r, 0, size[1]-1);
			float* row = results[k][y];
			for (int x = 0; x < size[0]; x++) {
				tl[0] = Clamp<int>(x-r, 0, size[0]-1);
				br[0] = Clamp<int>(x+r, 0, size[0]-1);
				row[x] = integ.Sum(tl, br) / ((br[0]-tl[0]+1)*(br[1]-tl[1]+1));
			}
		}
		k++;

		// Compute neighbour features
		if (nbrs) {
			for (int t = -1; t <= 1; t += 2) {
				// Horizontal shift
				results[k].Resize(size[1], size[0]);
				ShiftVert(accumulated, results[k], roundi(t*r));
				k++;

				// Vertical shift
				results[k].Resize(size[1], size[0]);
				ShiftHoriz(accumulated, results[k], roundi(t*r));
				k++;
			}
		}
	}



	PhotometricFeatures::PhotometricFeatures() {
	}

	PhotometricFeatures::PhotometricFeatures(const string& config) {
		Configure(config);
	}

	void PhotometricFeatures::Configure(const string& config) {
		components.clear();

		string feature_set;
		if (config == "default") {
			feature_set = *gvDefaultFeatureSet;
		} else {
			feature_set = config;
		}

		// Build a list of components to be included when "all" is specified
		set<string> normal;
		normal.insert("rgb");
		normal.insert("hsv");
		normal.insert("gabor");
		normal.insert("sweeps");
		normal.insert("accum_sweeps");
		normal.insert("nbr_sweeps");

		// Build the set of "special" components
		set<string> special;
		special.insert("gt");

		// Build the set of all available components
		set<string> available;
		set_union(normal.begin(), normal.end(),
							special.begin(), special.end(),
							inserter(available, available.begin()));
		
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

	bool PhotometricFeatures::IsActive(const string& component) {
		return components.find(component) != components.end();
	}

	void PhotometricFeatures::Compute(const PosedImage& image) {
		Compute(image, NULL);
	}

	void PhotometricFeatures::Compute(const PosedImage& image, const MatI* gt_orients) {
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
					Colors::RGB2HSV(imrow[x].r, imrow[x].g, imrow[x].b, h, s, v);
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
			CHECK_EQ(image.nx() % div, 0)
				<< "Image dimensions must be exactly divisible by 2^num_scales";
			CHECK_EQ(image.ny() % div, 0)
				<< "Image dimensions must be exactly divisible by 2^num_scales";

			gabor.Run(m);
			gabor_features.resize(gabor.responses.size());
			for (int i = 0; i < gabor.responses.size(); i++) {
				const MatF& response = *gabor.responses[i];
				gabor_features[i].Resize(image.ny(), image.nx());
				Upsample(response, image.ny()/response.Rows(), gabor_features[i]);
				features.push_back(&gabor_features[i]);

				int orient_index = i/gabor.num_scales;
				float orient = orient_index*180.0/gabor.num_orients;  // convert to degrees
				int scale = 1<<(i%gabor.num_orients);
				feature_strings.push_back(fmt("Gabor orient=%.1f scale=%dx", orient, scale));
			}
		}
		CHECK_EQ(features.size(), feature_strings.size());

		//
		// Line sweeps
		//
		CHECK(!IsActive("accum_sweeps") || IsActive("sweeps"))
			<< "If 'accum_sweeps' is selected then 'sweeps' must also be.";
		CHECK(!IsActive("nbr_sweeps") || IsActive("accum_sweeps"))
			<< "If 'nbr_sweeps' is selected then 'accum_sweeps' must also be.";

		if (IsActive("sweeps")) {
			line_detector.Compute(image);
			line_sweeper.Compute(image, line_detector.detections);
			sweep_features.resize(3);
			for (int i = 0; i < 3; i++) {
				sweep_features[i].Resize(image.ny(), image.nx());
				features.push_back(&sweep_features[i]);
				feature_strings.push_back(fmt("Line sweeps for orient %d", i));
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

			if (IsActive("accum_sweeps")) {
				bool include_nbrs = IsActive("nbr_sweeps");
				string namepat = "Accumulated sweeps (orientation %d, radius %d, centre)";
				string namepatnbr = "Accumulated sweeps (orientation %d, radius %d, nbr %d)";
				int base_radius = *gvFirstWindowForSweeps;
				int num_scales = *gvNumScalesForSweeps;
				for (int i = 0; i < 3; i++) {
					accum[i].Prepare(sweep_features[i]);
					int radius = 1;
					for (int j = 0; j < num_scales; j++) {
						radius *= base_radius;
						accum[i].Compute(radius, include_nbrs);
						feature_strings.push_back(fmt(namepat, i, radius));
						if (include_nbrs) {
							for (int k = 0; k < 4; k++) {
								feature_strings.push_back(fmt(namepatnbr, i, radius, k));
							}
						}
					}
					BOOST_FOREACH(const MatF& m, accum[i].results) {
						features.push_back(&m);
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
				<< "'gt' was requested in the feature set "
				<< "but no ground truth provided to PhotometricFeatures::Configure()";
			CHECK_SAME_SIZE(*gt_orients, image);
			gt_features.resize(3);
			for (int i = 0; i < 3; i++) {
				gt_features[i].Resize(image.ny(), image.nx());
				features.push_back(&gt_features[i]);
				feature_strings.push_back(fmt("Ground truth for orient %d", i));
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
