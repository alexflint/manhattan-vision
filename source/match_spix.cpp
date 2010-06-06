#include <ext/algorithm>

#include <VW/Image/imagecopy.h>
#include <VW/Image/imagecopy.tpp>
#include <VW/Utils/timer.h>

#include "vanishing_points.h"
#include "common_types_vw.h"
#include "fhsegmenter.h"
#include "timer.h"
#include "misc.h"
#include "kmeans.h"
#include "table.tpp"
#include "canny.h"
#include "range_utils.tpp"

// Computes a manhattan distance transform from a segmentation
class ManhattanDistTransform {
public:
	MatI dists;

	void Compute(const MatI& seg) {
		const int& w = seg.Cols();
		const int& h = seg.Rows();
		dists.Resize(seg.Rows(), seg.Cols());

		// Forwards and backwards horizontal scans
		for (int r = 0; r < h; r++) {
			const int* segrow = seg[r];
			int* distrow = dists[r];
			distrow[0] = 0;
			for (int c = 1; c < w; c++) {
				if (segrow[c] == segrow[c-1]) {
					distrow[c] = distrow[c-1]+1;
				} else {
					distrow[c] = 0;
				}
			}
			distrow[w-1] = 0;
			for (int c = w-2; c >= 0; c--) {
				if (segrow[c] == segrow[c+1]) {
					distrow[c] = min(distrow[c], distrow[c+1]+1);
				} else {
					distrow[c] = 0;
				}
			}
		}

		// Forwards vertical scan. Process row-by-row for cache efficiency
		fill_n(dists[0], w, 0);
		for (int r = 1; r < h; r++) {
			const int* segrow = seg[r];
			const int* prevsegrow = seg[r-1];
			int* distrow = dists[r];
			int* prevdistrow = dists[r-1];
			for (int c = 0; c < w; c++) {
				if (segrow[c] == prevsegrow[c]) {
					distrow[c] = min(distrow[c], prevdistrow[c]+1);
				} else {
					distrow[c] = 0;
				}
			}
		}

		// Backwards vertical scan. Process row-by-row for cache efficiency
		fill_n(dists[h-1], w, 0);
		for (int r = h-2; r >= 0; r--) {
			const int* segrow = seg[r];
			const int* prevsegrow = seg[r+1];
			int* distrow = dists[r];
			int* prevdistrow = dists[r+1];
			for (int c = 0; c < w; c++) {
				if (segrow[c] == prevsegrow[c]) {
					distrow[c] = min(distrow[c], prevdistrow[c]+1);
				} else {
					distrow[c] = 0;
				}
			}
		}
	}

	void OutputDistViz(const string& filename) {
		WriteMatrixImageRescaled(filename, dists);
	}
};


class NhdPixelFeatures {
public:
	vector<VecD> features;
	scoped_ptr<Table<2,VecD> > feature_map;

	void Compute(const ImageRGB<byte>& image, const ImageHSV<byte>& imagehsv) {
		const int& w = image.GetWidth();
		const int& h = image.GetHeight();

		feature_map.reset(new Table<2,VecD>(h, w));
		for (int r = 0; r < h; r++) {
			const PixelRGB<byte>* row = image[r];
			for (int c = 0; c < w; c++) {
				VecD& ftr = (*feature_map)(r, c);
				ftr.Resize(3);
				if (r > 0 && r < h-1 && c > 0 && c < w-1) {
					int i = 0;
					ftr[i++] = imagehsv[r][c].h;
					ftr[i++] = imagehsv[r][c].s;
					ftr[i++] = imagehsv[r][c].v;
					/*ftr[i++] = imagehsv[r-1][c-1].v - ftr[1];
					ftr[i++] = imagehsv[r-1][c].v - ftr[1];
					ftr[i++] = imagehsv[r-1][c+1].v - ftr[1];
					ftr[i++] = imagehsv[r][c-1].v - ftr[1];
					ftr[i++] = imagehsv[r][c+1].v - ftr[1];
					ftr[i++] = imagehsv[r+1][c-1].v - ftr[1];
					ftr[i++] = imagehsv[r+1][c].v - ftr[1];
					ftr[i++] = imagehsv[r+1][c+1].v - ftr[1];*/
				} else {
					ftr.Fill(0);
				}

				features.push_back(ftr);
			}
		}
	}
};


template <typename PixelFeatures>
class TextonLabeller {
public:
	MatI label_map;
	vector<VecD> textons;
	PixelFeatures pixel_ftrs;

	void Compute(const ImageRGB<byte>& image, const ImageHSV<byte>& imagehsv) {
		const int& w = image.GetWidth();
		const int& h = image.GetHeight();
		label_map.Resize(h, w);

		// Compute pixel-wise features
		pixel_ftrs.Compute(image, imagehsv);

		// Run K-means to generate textons
		vector<VecD> points;
		const vector<VecD>& ftrs = pixel_ftrs.features;
		random_sample_n(ftrs.begin(), ftrs.end(), back_inserter(points), 5000);
		VecI labels(points.size());
		KMeans::Estimate(points, 20, textons, labels);

		// Label the pixels
		for (int r = 0; r < h; r++) {
			for (int c = 0; c < w; c++) {
				const VecD& ftr = (*pixel_ftrs.feature_map)(r, c);
				double mindist = INFINITY;
				for (int i = 0; i < textons.size(); i++) {
					const double dist = VectorSSD(textons[i], ftr);
					if (dist < mindist) {
						mindist = dist;
						label_map[r][c] = i;
					}
				}
			}
		}
	}
};

class HsvBinFeatures {
public:
	MatI label_map;

	void Compute(const ImageRGB<byte>& image,
							 const ImageHSV<byte>& imagehsv) {
		const int kNumBins = 5;
		const int& w = image.GetWidth();
		const int& h = image.GetHeight();
		label_map.Resize(h, w);
		for (int r = 0; r < h; r++) {
			for (int c = 0; c < w; c++) {
				label_map[r][c] = static_cast<int>
					(imagehsv[r][c].h * kNumBins / 256);
			}
		}
	}
};



template <typename PixelFeatures>
class HistSegmentFeatures {
public:
	PixelFeatures pixel_ftrs;
	vector<VecF> features;
	MatF hists;

	void Compute(const ImageRGB<byte>& imagergb,
							 const ImageHSV<byte>& imagehsv,
							 const MatI& seg,
							 int num_segments) {
		const int& w = imagergb.GetWidth();
		const int& h = imagergb.GetHeight();
		// Compute pixel features
		pixel_ftrs.Compute(imagergb, imagehsv);

		// Build histograms over labels for each segment
		hists.Resize(num_segments, pixel_ftrs.label_map.MaxValue()+1);
		hists.Fill(0);
		for (int r = 0; r < h; r++) {
			const int* labelrow = pixel_ftrs.label_map[r];
			const int* segrow = seg[r];
			for (int c = 0; c < w; c++) {
				hists[ segrow[c] ][ labelrow[c] ]++;
			}
		}

		// Normalize the histograms
		features.resize(num_segments);
		for (int i = 0; i < num_segments; i++) {
			features[i] = hists.GetRow(i);
			features[i] /= features[i].Sum();
		}
	}

	void OutputLabelViz(const string& filename) const {
		WriteMatrixImageRescaled(filename, pixel_ftrs.label_map);
	}
};

class OrientSegmentFeatures {
 public:
	MatI segmentation;
	int num_segments;

	Gradients gradients;
	ManhattanDistTransform distxform;

	ImageF imagemono;

	MatI mask;

	VecF seg_x;
	VecF seg_y;
	VecF seg_dx;
	VecF seg_dy;
	VecI seg_sizes;

	vector<VecF> features;

	void Compute(const ImageRGB<byte>& imagergb,
							 const MatI& seg,
							 int num_segs) {
		const int& w = imagergb.GetWidth();
		const int& h = imagergb.GetHeight();
		segmentation = seg;
		num_segments = num_segs;
		
		// Compute gradients
		ImageCopy(imagergb, imagemono);
		gradients.Compute(imagemono);
		distxform.Compute(seg);

		// Compute average gradient in each segment
		mask.Resize(h, w);
		mask.Fill(0);
		seg_x.Resize(num_segments, 0.0);
		seg_y.Resize(num_segments, 0.0);
		seg_dx.Resize(num_segments, 0.0);
		seg_dy.Resize(num_segments, 0.0);
		seg_sizes.Resize(num_segments, 0);
		for (int r = 0; r < h; r++) {
			const int* segrow = seg[r];
			const PixelF* dxrow = gradients.diffx[r];
			const PixelF* dyrow = gradients.diffy[r];
			const int* distrow = distxform.dists[r];
			for (int c = 0; c < w; c++) {
				if (distrow[c] >= 2) {
					const int seg = segrow[c];
					seg_x[seg] += c;
					seg_y[seg] += r;
					seg_dx[seg] += dxrow[c].y;  // "y" just means the value of the pixel
					seg_dy[seg] += dyrow[c].y;
					seg_sizes[seg]++;
					mask[r][c] = 1;
				}
			}
		}

		// Compose features and normalize
		features.resize(num_segments);
		for (int i = 0; i < num_segments; i++) {
			if (seg_sizes[i] > 0) {
				seg_x[i] /= seg_sizes[i];
				seg_y[i] /= seg_sizes[i];
				seg_dx[i] /= seg_sizes[i];
				seg_dy[i] /= seg_sizes[i];
			}
			features[i] = MakeVector<2,float>(seg_dx[i], seg_dy[i]);
		}
	}

	void OutputGradientViz(const string& filename,
												 const ImageRGB<byte>& image) const {
		const int& w = image.GetWidth();
		const int& h = image.GetHeight();

		// Draw segment boundaries
		ImageRGB<byte> canvas;
		const MatI& segmap = segmentation;
		ImageCopy(image, canvas);
		for (int r = 0; r < h; r++) {
			for (int c = 0; c < w; c++) {
				if (distxform.dists[r][c] == 0) {
					canvas[r][c] = BrightColors::Get(segmap[r][c]);
				}
			}
		}

		// Draw segment orientation
		for (int i = 0; i < num_segments; i++) {
			if (seg_sizes[i] > 40) {
				const double norm = 10.0 / sqrt(seg_dx[i]*seg_dx[i] + seg_dy[i]*seg_dy[i]);
				Vector<2> a = makeVector(seg_x[i], seg_y[i]);
				Vector<2> b = makeVector(seg_x[i]+seg_dx[i]*norm, seg_y[i]+seg_dy[i]*norm);
				DrawSpot(canvas, BrightColors::Get(i), a, 1);
				DrawLineClipped(canvas, a, b, BrightColors::Get(i));
			}
		}
		WriteImage("out/segorients.png", canvas);
	}
};

void WriteHueOnly(const string& filename,
									ImageHSV<byte>& image) {
	ImageHSV<byte> clone;
	ImageCopy(image, clone);
	for (int r = 0; r < clone.GetHeight(); r++) {
		for (int c = 0; c < clone.GetWidth(); c++) {
			clone[r][c].s = 255;
		}
	}
	WriteImage(filename, clone);
}

int main(int argc, char **argv) {
	InitVars(argc, argv, "segment.cfg");
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" INPUT"<<endl;
		exit(-1);
	}

	// Read image
	ImageBundle image(argv[1]);
	const int& w = image.nx();
	const int& h = image.ny();
	image.BuildHSV();

	// Convert to HSV and mono
	WriteHueOnly("out/h_only.png", image.hsv);

	// Segment
	FHSegmenter segmenter;
	TIMED("Segment")
		segmenter.Compute(image);
	const int num_segments = segmenter.num_segments;
	DLOG << "Generated " << num_segments << " segments";
	segmenter.OutputSegViz("out/segmentation.png");

	// Compute features for each segment
	HistSegmentFeatures<TextonLabeller<NhdPixelFeatures> > segftrs;
	segftrs.Compute(image.rgb, image.hsv, segmenter.segmentation, num_segments);
	segftrs.OutputLabelViz("out/label_map.png");

	// Compute similarity matrix
	MatF similarity(num_segments, num_segments);
	for (int i = 0; i < num_segments; i++) {
		similarity[i][i] = 0;
		for (int j = i+1; j < num_segments; j++) {
			const double ssd = VectorSSD(segftrs.features[i], segftrs.features[j]);
			similarity[i][j] = min(1.0/sqrt(ssd), 10);
			similarity[j][i] = similarity[i][j];
		}
	}
	similarity /= similarity.MaxValue();
	WriteMatrixImageRescaled("out/similarity.png", similarity);

	// Generate some vizualizations
	const int kNumViz = 15;
	ImageRGB<byte> viz(w, h);
	vector<PixelRGB<byte> > colors(num_segments);

	vector<int> segment_indices;
	sorted_order(segmenter.segment_sizes, segment_indices, greater<int>());

	for (int i = 0; i < kNumViz; i++) {
		// Pick a random pixel and use the segment at that pixel so that
		// larger segments are more likely to be chosen
		const int root = segment_indices[i];
		DREPORT(root);

		// Compute colors for each other segment
		for (int j = 0; j < num_segments; j++) {
			if (j == root) {
				colors[j].Set(255, 0, 0);
			} else {
				const int v = similarity[j][root] * 255;
				colors[j].Set(v, v, v);
			}
		}

		// Generate the vizualization
		for (int r = 0; r < h; r++) {
			PixelRGB<byte>* vizrow = viz[r];
			const int* segrow = segmenter.segmentation[r];
			for (int c = 0; c < w; c++) {
				vizrow[c] = colors[segrow[c]];
			}
		}
		WriteImage("out/sim"+PaddedInt(i,2)+".png", viz);
	}

	return 0;
}
