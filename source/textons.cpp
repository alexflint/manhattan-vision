
#include <stdlib.h>
#include <time.h>

#include <iomanip>
#include <functional>
#include <ext/algorithm>
#include <ext/numeric>
#include <ext/functional>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <VW/Image/pixelconversions.h>

#include "common_types.h"
#include "textons.h"
#include "filters.h"
#include "kmeans.h"
#include "progress_reporter.h"

#include <VW/Image/imagecopy.tpp>

#include "io_utils.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	// Represents the type of color information included in feature
	// vectors
	enum ColorInfo {
		kNone = 0,
		kMono = 1,
		kRGB = 2,
		kHSV = 3
	};

	// Num scales in the Gabor filter bank
	const lazyvar<int> gvNumScales("Textons.Filters.NumScales");
	// Num orientations in the Gabor filter bank
	const lazyvar<int> gvNumOrients("Textons.Filters.NumOrients");
	// Vocabulary file for textons
	const lazyvar<string> gvVocabFile("Textons.VocabFile");
	// Strategy for filtering (CPU, CPUParallel, or GPU)
	const lazyvar<string> gvFilterStrategy("Textons.FilterStrategy");

	lazyvar<int> gvColorInfo("Textons.Features.ColorInfo");
	lazyvar<float> gvGaborWeight("Textons.Features.GaborWeight");
	lazyvar<float> gvMonoWeight("Textons.Features.MonoWeight");
	lazyvar<float> gvRWeight("Textons.Features.RWeight");
	lazyvar<float> gvGWeight("Textons.Features.GWeight");
	lazyvar<float> gvBWeight("Textons.Features.BWeight");
	lazyvar<float> gvHWeight("Textons.Features.HWeight");
	lazyvar<float> gvSWeight("Textons.Features.SWeight");
	lazyvar<float> gvVWeight("Textons.Features.VWeight");


	TextonVocab::TextonVocab() {
	}

	TextonVocab::TextonVocab(const string& filename) {
		Load(filename);
	}

	void TextonVocab::Save(const string& filename) {
		WriteVecs(filename, words);
	}

	void TextonVocab::Load(const string& filename) {
		ReadVecs(filename, words);
	}

	void TextonVocab::Compute(const vector<ImageBundle*>& images) {
		input = &images;

		TextonFeatures ftrgen;
		ProgressReporter gen_prog(images.size(), "Generating features");
		for (int i = 0; i < images.size(); i++) {
			// Run the filters
			ftrgen.Compute(*images[i]);

			// Compute the features
			Vector<> feature(ftrgen.FeatureLen());;
			for (int y = 0; y < images[i]->ny(); y+=3) {
				for (int x = 0; x < images[i]->nx(); x+=3) {
					ftrgen.Get(x, y, feature);
					sampled_features.push_back(asVNL(feature));
				}
			}
			gen_prog.Increment();
		}

		// Run K-means to learn the texton exemplars
		DLOG << "Sampled " << sampled_features.size() << " features";
		DLOG << "Running K-means...";
		vector<VecD> exemplars;
		VecI parents(sampled_features.size());
		KMeans::Estimate(sampled_features, 15, exemplars, parents);

		// Copy wrods to Toon format
		words.clear();
		Vector<>(*f)(const VecD&) = &asToon<double>;
		transform_all_into(exemplars, words, f);
	}



	// Read these directly for efficiency and thread safety
	TextonFeatures::TextonFeatures() : feature_size_(-1) {
		InitVars();
	}

	TextonFeatures::TextonFeatures(const ImageBundle& image) : feature_size_(-1) {
		InitVars();
		Compute(image);
	}

	void TextonFeatures::InitVars() {
		// These vars are used in TextonFeatures::Get so cache them for
		// efficiency and thread safety.
		color_info = *gvColorInfo;
		gabor_weight = *gvGaborWeight;
		mono_weight = *gvMonoWeight;
		r_weight = *gvRWeight;
		g_weight = *gvGWeight;
		b_weight = *gvBWeight;
		h_weight = *gvHWeight;
		s_weight = *gvSWeight;
		v_weight = *gvVWeight;
	}
		

	void TextonFeatures::Compute(const ImageBundle& image) {
		input = &image;

		// Convert to matrix
		image.BuildMono();
		MatF m(image.ny(), image.nx());
		for (int y = 0; y < image.ny(); y++) {
			const PixelF* in = image.mono[y];
			float* out = m[y];
			for (int x = 0; x < image.nx(); x++) {
				out[x] = in[x].y;
			}
		}

		// Configure the filter bank
		if (filters.filterbank.get() == NULL) {
			filters.Configure(*gvNumScales, *gvNumOrients);
		}

		// Run the filter bank
		if (*gvFilterStrategy == "CPU") {
			filters.Run(m);
		} else if (*gvFilterStrategy == "CPUParallel") {
			filters.RunParallel(m);
		} else if (*gvFilterStrategy == "GPU") {
			filters.RunOnHardware(m);
		} else {
			DLOG << "Unknown filter strategy: " << *gvFilterStrategy << endl;
			exit(-1);
		}

		// Compute feature length for efficiency and thread safety
		// (ProcessRows calls FeatureLen in parallel, which must not read
		// lazy vars)
		switch (color_info) {
		case kNone: feature_size_ = filters.responses.size(); break;
		case kMono: feature_size_ = filters.responses.size() + 1; break;
		case kRGB: feature_size_ = filters.responses.size() + 3; break;
		case kHSV: feature_size_ = filters.responses.size() + 3; break;
		}
	}

	int TextonFeatures::FeatureLen() const {
		return feature_size_;
	}

	void TextonFeatures::Get(int x, int y, Vector<>& ftr) const {
		// Don't use any gabor features here
		assert(ftr.size() == FeatureLen());

		const int n = filters.responses.size();
		int px = x;
		int py = y;
		for (int i = 0; i < n; i++) {
			const MatF& cur = *filters.responses[i];
			if (i > 0 && cur.Rows() < filters.responses[i-1]->Rows()) {
				py /= 2;
				px /= 2;
			}
			ftr[i] = gabor_weight * cur[py][px];
		}

		switch (color_info) {
		case kMono: {
			// TODO: what is ImageMono<float> normalized to, what should we
			// divide by??
			ftr[n] = mono_weight * input->mono[y][x].y;
			break;
		}
		case kRGB: {
			const PixelRGB<byte>& p = input->rgb[y][x];
			ftr[n] = r_weight * p.r / 127.5;
			ftr[n+1] = g_weight * p.g / 127.5;
			ftr[n+2] = b_weight * p.b / 127.5;
			break;
		}
		case kHSV: {
			float h, s, v;
			const PixelRGB<byte>& q = input->rgb[y][x];
			VW::ImageConversions::RGB2HSV(q.r, q.g, q.b, h, s, v);
			ftr[n] = h_weight * h / 127.5;
			ftr[n+1] = s_weight * s / 127.5;
			ftr[n+2] = v_weight * v / 127.5;
			break;
		}
		case kNone:
			break;
		}
	}





	TextonMap::TextonMap() {
	}

	TextonMap::TextonMap(const ImageBundle& image) {
		Compute(image);
	}

	void TextonMap::Compute(const ImageBundle& image) {
		Reset(image);
		ProcessRows(0, image.ny()-1);
	}


	void TextonMap::ComputeParallel(const ImageBundle& image) {
		const int kNumThreads = 4;
		Reset(image);
		ParallelPartition(image.ny(),
											kNumThreads,
											bind(&TextonMap::ProcessRows, ref(*this), _1, _2));
	}

	void TextonMap::Reset(const ImageBundle& image) {
		input = &image;

		// Load vocabulary
		if (vocab.words.empty()) {
			vocab.Load(*gvVocabFile);
		}

		// Resize buffers
		map.Resize(image.ny(), image.nx());
		texton_counts.Resize(vocab.words.size(), 0);

		// Compute features
		features.Compute(image);
	}

	void TextonMap::ProcessRows(const int r1, const int r2) {
		Vector<> ftr(features.FeatureLen());
		for (int y = r1; y <= r2; y++) {
			int* maprow = map[y];
			for (int x = 0; x < map.Cols(); x++) {
				// Generate the feature vector for this pixel
				features.Get(x, y, ftr);

				// Find the nearest texon
				double mindist = INFINITY;
				for (int i = 0; i < vocab.words.size(); i++) {
					const double dist = norm_sq(ftr - vocab.words[i]);
					if (dist < mindist) {
						mindist = dist;
						maprow[x] = i;
					}
				}
				texton_counts[maprow[x]]++;
			}
		}
	}


	void TextonMap::Load(const string& filename) {
		ReadMatrix(filename, map);
		texton_counts.Resize(map.MaxValue()+1, 0);
		for (int y = 0; y < map.Rows(); y++) {
			const int* row = map[y];
			for (int x = 0; x < map.Cols(); x++) {
				texton_counts[row[x]]++;
			}
		}
	}

	void TextonMap::Save(const string& filename) const {
		WriteMatrix(filename, map);
	}

	void TextonMap::DrawTextonViz(ImageRGB<byte>& canvas,
																const int texton) const {
		ResizeImage(canvas, map.Cols(), map.Rows());
		ImageCopy(input->rgb, canvas);
		PixelRGB<byte> highlight(255, 0, 0);
		for (int y = 0; y < canvas.GetHeight(); y++) {
			const int* maprow = map[y];
			PixelRGB<byte>* imrow = canvas[y];
			for (int c = 0; c < canvas.GetWidth(); c++) {
				if (maprow[c] == texton) {
					imrow[c] = highlight;
				}
			}
		}
	}


	void TextonMap::OutputTextonViz(const string& filename,
																	const int texton) const {
		ImageRGB<byte> canvas;
		DrawTextonViz(canvas, texton);
		WriteImage(filename, canvas);
	}

	void TextonMap::DrawMapViz(ImageRGB<byte>& canvas) const {
		ResizeImage(canvas, map.Cols(), map.Rows());
		DrawSegmentation(map, canvas);
	}

	void TextonMap::OutputMapViz(const string& filename) const {
		ImageRGB<byte> canvas;
		DrawMapViz(canvas);
		WriteImage(filename, canvas);
	}
}
