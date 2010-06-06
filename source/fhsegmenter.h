#pragma once

#include "common_types.h"
#include "union_find.h"
#include "image_bundle.h"

namespace indoor_context {

struct Edge {
	int src;
	int dest;
	float weight;
	inline Edge() {}
	inline Edge(int s, int d, float w) : src(s), dest(d), weight(w) { }
	bool operator<(const Edge& other) const {
		return weight < other.weight;
	}
};

class FHSegmenter {
public:
	// The smoothed image, or empty if FHSegmenter.SmoothingSigma is 0
	ImageF smoothed;
	// The final segmentation. Each pixel is labelled with its segment
	// index (ranges from 0 to num_segments-1)
	MatI segmentation;
	// The number of segments.
	int num_segments;
	// The size of each segment
	vector<int> segment_sizes;
	// Internal representation included here to allow memory re-use
	// across invokations
	VecF seg_maxedge;
	VecI seg_labels;
	VecI seg_counts;  // not to be confused with seg_sizes
	UnionFind uf;
	vector<Edge> edges;

	// Initialize the segmenter empty
	FHSegmenter();
	// Initialize the segmenter and execute it for the given image
	FHSegmenter(const ImageBundle& input);
	// Run the segmentation, store the result in this->segmentation
	int Compute(const ImageBundle& input);
	// Draw the segmentation
	void OutputSegViz(const string& filename) const;
};

}
