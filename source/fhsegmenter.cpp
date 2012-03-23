#include <algorithm>
#include <numeric>
#include <ext/numeric>
#include <stack>

//#include <parallel/algo.h>

#include <boost/range.hpp>
#include <boost/thread.hpp>

#include "common_types.h"
#include "fhsegmenter.h"
#include "union_find.h"
#include "timer.h"
#include "filters.h"
#include "image_bundle.h"
#include "image_utils.tpp"

#include <boost/bind.hpp>

namespace indoor_context {

	const lazyvar<float> gvK("FHSegmenter.K");
	const lazyvar<float> gvMinSize("FHSegmenter.MinSize");
	const lazyvar<float> gvMinDiff("FHSegmenter.MinDiff");
	const lazyvar<float> gvSigma("FHSegmenter.SmoothingSigma");

	FHSegmenter::FHSegmenter() {
	}

	FHSegmenter::FHSegmenter(const ImageBundle& image) {
		Compute(image);
	}

	int FHSegmenter::Compute(const ImageBundle& rawimage) {
		DLOG << "Segmenting image";
		SCOPED_INDENT;

		rawimage.BuildMono();
		const int& w = rawimage.nx();
		const int& h = rawimage.ny();

		// Convert to matrix
		rawimage.BuildMono();
		MatF m(rawimage.ny(), rawimage.nx());
		for (int y = 0; y < rawimage.ny(); y++) {
			const PixelF* in = rawimage.mono[y];
			float* out = m[y];
			for (int x = 0; x < rawimage.nx(); x++) {
				out[x] = in[x].y;
			}
		}

		// Lazily resize the containers
		seg_counts.Resize(w*h);
		seg_maxedge.Resize(w*h);
		seg_labels.Resize(w*h);
		segmentation.Resize(h, w);
		edges.resize(2 * (w-1) * (h-1));

		// Smooth if required
		if (*gvSigma > 0) {
			TIMED("Smoothing") SmoothGaussian(*gvSigma, m, smoothed);
		}
		const MatF& image = *gvSigma > 0 ? smoothed : m;

		// Join row neighbours closer than min_diff for efficiency
		int count = 0;
		TIMED("Horiz join") for (int r = 0; r < h; r++) {
			const float* row = image[r];
			int* segrow = segmentation[r];
			for (int c = 0; c < w; c++) {
				if (c > 0 && fabsf(row[c] - row[c-1]) <= *gvMinDiff) {
					segrow[c] = segrow[c-1];
					seg_counts[segrow[c]]++;
				} else {
					segrow[c] = count++;
					seg_counts[segrow[c]] = 1;
				}
			}
		}

		// Extract edges
		vector<Edge>::iterator e = edges.begin();
		TIMED("Get edges") for (int r = 0; r < h-1; r++) {
			const float* row = image[r];
			const float* nextrow = image[r+1];
			const int* segrow = segmentation[r];
			const int* nextsegrow = segmentation[r+1];
			for (int c = 0; c < w-1; c++) {
				// Add horizontal edge
				if (segrow[c] != segrow[c+1]) {
					e->src = segrow[c];
					e->dest = segrow[c+1];
					e->weight = fabsf(row[c] - row[c+1]);
					e++;
				}
				e->src = segrow[c];
				e->dest = nextsegrow[c];
				e->weight = fabsf(row[c] - nextrow[c]);
				e++;
			}
		}
		const int num_edges = e - edges.begin();

		// Sort edges by weight
		TIMED("Sort") sort(edges.begin(), edges.begin()+num_edges);

		// Agglomerate until done
		uf.Reset(count);
		fill_n(seg_maxedge.begin(), count, 0);

		TIMED("Agglomerate")
			for (int i = 0; i < num_edges; i++) {
				const int a = uf.GetGroup(edges[i].src);
				const int b = uf.GetGroup(edges[i].dest);
				if (a != b) {
					const float aint = seg_maxedge[a] + *gvK / seg_counts[a];
					const float bint = seg_maxedge[b] + *gvK / seg_counts[b];
					if (edges[i].weight <= min(aint, bint)) {
						const int c = uf.Merge(a, b);
						seg_counts[c] = seg_counts[a] + seg_counts[b];
						seg_maxedge[c] = edges[i].weight;
					}
				}
			}

		// Remove segments that are too small and from the segmentation
		num_segments = 0;
		fill_n(seg_labels.begin(), count, -1);
		TIMED("Cleanup") for (int r = 0; r < h; r++) {
			int* segrow = segmentation[r];
			for (int c = 0; c < w; c++) {
				// Recall we are *updating* the segmentation from above
				const int g = uf.GetGroup(segrow[c]);
				if (c == 0 || seg_counts[g] >= *gvMinSize) {
					if (seg_labels[g] == -1) {
						seg_labels[g] = num_segments++;
						segment_sizes.push_back(seg_counts[g]);
					}
					segrow[c] = seg_labels[g];
				} else {
					segrow[c] = segrow[c-1];
				}
			}
		}

		return num_segments;
	}

	void FHSegmenter::OutputSegViz(const string& filename) const {
		ImageRGB<byte> output(segmentation.Cols(), segmentation.Rows());
		DrawSegmentation(segmentation, output);
		WriteImage(filename, output);
	}

}
