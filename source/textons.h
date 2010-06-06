#pragma once

#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "filters.h"

namespace indoor_context {
	// Represents a set of texton words (i.e. cluster centres)
	class TextonVocab {
	public:
		// The cluster centres in feature space
		vector<toon::Vector<> > words;
		// The features generated for the input images
		vector<VecD> sampled_features;
		// The images this vocab was generated from
		const vector<ImageBundle*>* last_input;

		// Initialize an empty vocabulary
		TextonVocab();
		// Initialize and load vocab from a file
		TextonVocab(const string& filename);
		// Save current words to a file
		void Save(const string& filename);
		// Load words from a file
		void Load(const string& filename);
		// Compute textons by clustering filter responses from several images
		void Compute(const vector<ImageBundle*>& images);
	};

	// Represents per-pixel image features.
	class TextonFeatures {
	public:
		// Filter bank
		GaborFilterBank filters;
		// Current image
		const ImageBundle* last_input;
		// Length of features (pre-computed for efficiency)
		int feature_size_;

		// These are used in TextonFeatures::Get so we copy them out of
		// gvars for efficiency and thread safety.
		int color_info;
		float gabor_weight, mono_weight;
		float r_weight, g_weight, b_weight;
		float h_weight, s_weight, v_weight;

		// Initialize empty
		TextonFeatures();
		// Initialize and compute features
		TextonFeatures(const ImageBundle& image);
		// Compute features
		void Compute(const ImageBundle& image);
		// Get the length of the features
		int FeatureLen() const;
		// Get the feature at (x,y)
		void Get(int x, int y, toon::Vector<>& out_ftr) const;
	private:
		// Load values from GVars (called during construction)
		void InitVars();
	};

	// Represents an image of texton indices
	class TextonMap {
	public:
		// The list of words
		TextonVocab vocab;
		// Filter responses
		TextonFeatures features;
		// Per-pixel texton map
		MatI map;
		// Number of pixels assigned to each texton
		VecI texton_counts;
		// Pointer to most recent input
		const ImageBundle* last_input;

		// Initialize empty
		TextonMap();
		// Initialize and compute texton map
		TextonMap(const ImageBundle& image);
		// Compute the texton map for an input image
		void Compute(const ImageBundle& image);
		// Compute the texton map for an input image
		void ComputeParallel(const ImageBundle& image);

		// Save current texton map and frequency table to a file
		void Save(const string& filename) const;
		// Load the texton map and frequency table from a file
		void Load(const string& filename);

		// Draw the segmentation map
		void DrawMapViz(ImageRGB<byte>& canvas) const;
		// Vizualize the texton map directly
		void OutputMapViz(const string& filename) const;
		// Vizualize the part of an image assigned to texton i
		void DrawTextonViz(ImageRGB<byte>& canvas, const int texton) const;
		// Vizualize the part of an image assigned to texton i
		void OutputTextonViz(const string& filename, const int texton) const;
	private:
		// Reset internal buffers
		void Reset(const ImageBundle& image);
		// Segment part of the image, used for parallelization
		void ProcessRows(int begin_row, int end_row);
	};

}
