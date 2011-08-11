#include <numeric>
#include <limits>

#include <VW/Image/imagehsv.h>
#include <VW/Image/imagecopy.h>

#include "common_types_vw.h"
#include "features.h"
#include "misc.h"
#include "filters.h"
#include "fhsegmenter.h"
#include "timer.h"
#include "vanishing_points.h"

static const int kNumHueBins = 5;
static const int kNumSatBins = 3;
static const int kNumFilters = 4;//12;

static const int kNumOrientBins = 8;
static const int kNumDistBins = 2;
static const float kNearRadius = 1.0;  // % of image size
static const float kFarRadius = 3.5;  // % of image size

// Threshold for declaring two lines "nearly parallel"
static const float kParallelThresh = M_PI/8.0;

static const float kVertVptThresh = 2.5;
static const float kHorizVptThresh = 1.25;

// Get entropy of histogram
float GetEntropy(const VecF& hist) {
	double h = 0.0;
	for (int i = 0; i < hist.size(); i++) {
		if (hist[i] > 0) {
			h += hist[i] * log(hist[i]);
		}
	}
	return h;
}

struct HehFeature {
	// Location features
	float area;
	float x_mean, y_mean;
	float x_10pct, x_90pct;  // 10th and 90th percentile
	float y_10pct, y_90pct;
	int horizon_status;  // 0=above, 1=below, 2=straddles
	float y_mean_wrt_horizon;  // mean y wrt horizon
	float y_10pct_wrt_horizon;  // 10th y percentile wrt horizon
	float y_90pct_wrt_horizon;  // 90th y percentile wrt horizon

	int num_child_superpixels;  // only for when we generate cluster
															// features

	// Color features
	float r_mean, g_mean, b_mean;  // RGB mean
	float h_mean, s_mean, v_mean;  // HSV mean
	VecF hue_hist;   // Histogram over hue
	VecF sat_hist;   // Histogram over saturation

	// Texture features
	VecF filter_means;  // Mean over filter responses
	VecF filter_max_hist; // For each filter, the number
																			// of pixels for which its
																			// response was greater than any
																			// other filter

	// Line features
	float pct_line_pix;  // num pixels belonging to a line segment / sqrt(area)
	float pct_parallel_lines;  // % nearly parallel lines
	float pct_isct_right;  // % intersections right of centre
	float pct_isct_bottom;  // % intersection below centre
	float isct_hist_entropy;  // entropy of histogram over intersections
	VecF isct_hist;  // line intersections, radially binned
	
	// Vanishing point features
	float pct_vert_vpt_pix;  // num line pixels belonging to vertical vanpt/sqrt(area)
	float pct_horiz_vpt_pix;  // num line pixels belonging to vertical vanpt/sqrt(area)
	float pct_vert_vpt;  // % line pixels with vertical vanpt membership
	float x_mean_wrt_hvpt;  // mean X minus X coord of horiz vanpt
	float x_10pct_wrt_hvpt;  // 10th X percentile minus X coord of horiz vanpt, if any
	float x_90pct_wrt_hvpt;  // 90th X percentile minus X coord of horiz vanpt, if any
	float y_mean_wrt_vert_vpt;  // mean Y minus Y coord of highest/lowest vanpt
	
	// Intermediate vars
	vector<float> xs, ys;  // X and Y coordinates of pixels in this segment
	vector<int> lines;  // index of lines that are present in this region
	VecI seen;  // whether we've seen each line segment in this region
	int num_line_pix;  // number of pixels belonging to a line

	// Initialize...
	HehFeature() : area(0),
								 x_mean(0),
								 y_mean(0),
								 x_10pct(0),
								 x_90pct(0),
								 y_10pct(0),
								 y_90pct(0),
								 horizon_status(0),
								 num_child_superpixels(0),

								 r_mean(0),
								 g_mean(0),
								 b_mean(0),
								 h_mean(0),
								 s_mean(0),
								 v_mean(0),
								 hue_hist(kNumHueBins, 0),
								 sat_hist(kNumSatBins, 0),

								 filter_means(kNumFilters, 0),
								 filter_max_hist(kNumFilters, 0),

								 pct_line_pix(0),
								 pct_parallel_lines(0),
								 isct_hist(kNumOrientBins * kNumDistBins, 0),
								 isct_hist_entropy(0),
								 pct_isct_right(0),
								 pct_isct_bottom(0),

								 pct_vert_vpt_pix(0),
								 pct_horiz_vpt_pix(0),
								 pct_vert_vpt(0),
								 x_mean_wrt_hvpt(0),
								 x_10pct_wrt_hvpt(0),
								 x_90pct_wrt_hvpt(0),

								 num_line_pix(0) { }
};	

ostream& operator<<(ostream& s, const HehFeature& ftr) {
	DREPORT_S(ftr.area, s);
	DREPORT_S(ftr.x_mean, s);
	DREPORT_S(ftr.y_mean, s);
	DREPORT_S(ftr.x_10pct, s);
	DREPORT_S(ftr.x_90pct, s);
	DREPORT_S(ftr.y_10pct, s);
	DREPORT_S(ftr.y_90pct, s);
	DREPORT_S(ftr.y_mean_wrt_horizon, s);
	DREPORT_S(ftr.y_10pct_wrt_horizon, s);
	DREPORT_S(ftr.y_90pct_wrt_horizon, s);
	
	DREPORT_S(ftr.r_mean, s);
	DREPORT_S(ftr.g_mean, s);
	DREPORT_S(ftr.b_mean, s);
	DREPORT_S(ftr.h_mean, s);
	DREPORT_S(ftr.s_mean, s);
	DREPORT_S(ftr.v_mean, s);
	DLOG << endl;

	DREPORT_S(ftr.hue_hist, s);
	DREPORT_S(ftr.sat_hist, s);
	DREPORT_S(ftr.filter_means, s);
	DREPORT_S(ftr.filter_max_hist, s);
	DLOG << endl;

	DREPORT_S(ftr.pct_line_pix, s);
	DREPORT_S(ftr.pct_parallel_lines, s);
	DREPORT_S(ftr.pct_isct_right, s);
	DREPORT_S(ftr.pct_isct_bottom, s);
	DREPORT_S(ftr.isct_hist_entropy, s);
	DREPORT_S(ftr.isct_hist, s);
	DLOG << endl;

	DREPORT_S(ftr.pct_vert_vpt_pix, s);
	DREPORT_S(ftr.pct_horiz_vpt_pix, s);
	DREPORT_S(ftr.pct_vert_vpt, s);
	DREPORT_S(ftr.x_mean_wrt_hvpt, s);
	DREPORT_S(ftr.x_10pct_wrt_hvpt, s);
	DREPORT_S(ftr.x_90pct_wrt_hvpt, s);
	DREPORT_S(ftr.y_mean_wrt_vert_vpt, s);
	DLOG << endl;

	DREPORT_S(ftr.xs.size(), s);
	DREPORT_S(ftr.ys.size(), s);
	DREPORT(ftr.num_line_pix);
	DLOG_N << "lines present: ";
	BOOST_FOREACH(const int line, ftr.lines) DLOG_N << line << " ";
	DLOG << endl;
}

// Generates features for the HEH context algorithm
class HehFeatureGen {
public:
	// Vanishing points detector
	VanishingPoints vpts;
	// The features we generated
	vector<HehFeature> features;

	// Compute features for an input image and segmentation
	void Compute(const ImageRGB<byte>& imagergb,
							 const ImageF& image,
							 const MatI& seg);
};

typedef VectorFixed<3,double> Vec3;

void HehFeatureGen::Compute(const ImageRGB<byte>& imagergb,
														const ImageF& image,
														const MatI& seg) {
	// General notes:

	//   We must always be careful to deal with normalized image
	//   coordinates, which range from 0 to 1 in both dimensions. This
	//   includes pixel locations, vanishing point locations, and line
	//   intersections.

	//   Any homogeneous vector can represent a point at infinity, which
	//   will manifest as an INF if we Project() it. This is fine so
	//   long as we keep track of which coordinates might potentially be
	//   INF and only do sensible things with them -- e.g. no sqrt(),
	//   atan2(), division between two variables that can both be INF,
	//   etc.

	const int w = image.GetWidth();
	const int h = image.GetHeight();
	const int image_size = w*h;

	// Run the filter bank
	// TODO: implement LM filters. Here we just use 12 Gabor filters.
	vector<shared_ptr<ImageF> > responses;
	FilterBank filterbank(1);  // *** should be 3 scales
	MakeGaborFilters(4, &filterbank.filters);
	filterbank.Run(image, &responses);

	//
	// Find lines and compute vanishing points
	//
	vpts.Compute(image);
	const int num_lines = vpts.lines.segments.size();
	const int num_vpts = vpts.vanpts.size();

	//
	// Pre-process the line segments
	//

	// Whether each pair of lines is nearly parallel
	MatI is_nearly_parallel(num_lines, num_lines);
	// Whether the intersection of each pair is right of center
	MatI isct_is_right(num_lines, num_lines);
	// Whether the intersection of each pair is below center
	MatI isct_is_bottom(num_lines, num_lines);
	// Histogram bin index for intersection of each pair, or -1 if outside
	MatI isct_bin(num_lines, num_lines);

	// Compute line intersections
	for (int i = 0; i < num_lines; i++) {
		const LineSegment& segi = *vpts.lines.segments[i];
		is_nearly_parallel[i][i] = 0;
		for (int j = 1; j < num_lines; j++) {
			const LineSegment& segj = *vpts.lines.segments[j];

			// Parallel ?
			float dtheta = segi.theta - segj.theta;
			dtheta = min(dtheta, M_PI-dtheta);
			is_nearly_parallel[i][j] = (dtheta <= kParallelThresh ? 1 : 0);

			if (is_nearly_parallel[i][j]) {
				// Intersection position relative to center
				Vec3 isct = Cross3D(segi.line, segj.line);
				isct[0] /= w;  // can be +/- INF
				isct[1] /= h;  // can be +/- INF
				const float eucl_x = Project(isct)[0];
				const float eucl_y = Project(isct)[1];
				isct_is_right[i][j] = eucl_x > 0.5;
				isct_is_bottom[i][j] = eucl_y > 0.5;

				// Compute distance from centre
				// don't take sqrt(radius_sqr) because radius_sqr can be INF
				const float radius_sqr = eucl_x*eucl_x + eucl_y*eucl_y;
				int rad_bin = -1;
				if (radius_sqr > kFarRadius*kFarRadius) {
					rad_bin = 1;
				} else if (radius_sqr > kNearRadius*kNearRadius) {
					rad_bin = 0;
				}

				// Compute orientation from centre
				if (rad_bin == -1) {
					isct_bin[i][j] = -1;
				} else {
					// Must do atan2 with homogeneous coords. It will stay sane in
					// the case of points-at-infinity
					float phi = atan2(isct[1] - 0.5*isct[2],
															isct[0] - 0.5*isct[2]);
					int phi_bin = static_cast<int>(phi*M_PI/kNumOrientBins)%kNumOrientBins;
					int bin = rad_bin * kNumOrientBins + phi_bin;
					isct_bin[i][j] = bin;
				}
			}

			is_nearly_parallel[j][i] = is_nearly_parallel[i][j];
			isct_is_right[j][i] = isct_is_right[i][j];
			isct_is_bottom[j][i] = isct_is_bottom[i][j];
			isct_bin[j][i] = isct_bin[i][j];
		}
	}

	// 
	// Pre-process vanishing points
	//
	enum VanPointCla {
		kVptNone,
		kVptHoriz,
		kVptVert
	};
	const float h_vpt_min = 0.5-kHorizVptThresh;
	const float h_vpt_max = 0.5+kHorizVptThresh;
	const float v_vpt_low = 0.5-kVertVptThresh;
	const float v_vpt_high = 0.5+kVertVptThresh;
	int num_hvpts = 0;
	float sum_hvpts_y = 0;
	vector<VectorFixed<2,float> > norm_vpts;  // can contain INF
	VecI vanpt_cla(vpts.vanpts.size(), kVptNone);
	COUNTED_FOREACH(int i, const VecD& vpt, vpts.vanpts) {
		const float eucl_x = Project(vpt)[0] / w;  // can be +/-INF
		const float eucl_y = Project(vpt)[1] / h;  // can be +/-INF

		// Classify vanishing point as horizontal, vertical, or none
		norm_vpts.push_back(MakeVector<2,float>(eucl_x, eucl_y));
		if (eucl_y > h_vpt_min &&	eucl_y < h_vpt_max) {
			vanpt_cla[i] = kVptHoriz;
			sum_hvpts_y += eucl_y;
			num_hvpts++;
		} else if (eucl_y < v_vpt_low || eucl_y > v_vpt_high) {
			vanpt_cla[i] = kVptVert;
		}
	}

	// Assume horizon is horizontal with Y coord equal to average of the
	// "horizontal" vanishing points
	float horizon_y = 0.5;
	if (num_hvpts > 0) {
		horizon_y = sum_hvpts_y / num_hvpts;
	}

	// Print vanpt clas
	DREPORT(vanpt_cla);



	// Initialize features
	int num_segments = seg.MaxValue()+1;
	features.resize(num_segments);
	for (int i = 0; i < features.size(); i++) {
		features[i].seen = VecI(num_lines, 0);
	}

	//
	// Compute sums over the image
	//
	for (int r = 0; r < h; r++) {
		const PixelRGB<byte>* imrgbrow = imagergb[r];
		const PixelF* imrow = image[r];
		const int* segrow = seg[r];
		const int* linerow = vpts.lines.segment_map[r];
		for (int c = 0; c < w; c++) {
			HehFeature& ftr = features[segrow[c]];

			// Shape
			ftr.xs.push_back(1.0*c/w);
			ftr.ys.push_back(1.0*r/h);

			// Color
			PixelHSV<byte> hsv;
			const PixelRGB<byte>& rgb = imrgbrow[c];
			ImageConversions::RGB2HSV(rgb.r, rgb.g, rgb.b,
																hsv.h, hsv.s, hsv.v);
			ftr.r_mean += rgb.r;
			ftr.g_mean += rgb.g;
			ftr.b_mean += rgb.b;
			ftr.h_mean += hsv.h;
			ftr.s_mean += hsv.s;
			ftr.v_mean += hsv.v;

			const int hbin = static_cast<int>(kNumHueBins * hsv.h / 256);
			const int sbin = static_cast<int>(kNumSatBins * hsv.s / 256);
			ftr.hue_hist[hbin]++;
			ftr.sat_hist[sbin]++;
			
			// Texture
			ImageRef pyrpos(c, r);
			int maxi;
			float maxv = -INFINITY;
			for (int i = 0; i < responses.size(); i++) {
				if (i > 0 && responses[i]->GetWidth() < responses[i-1]->GetWidth()) {
					pyrpos /= 2;
				}
				const float v = (*responses[i])[pyrpos].y;
				ftr.filter_means[i] += v;
				if (v > maxv) {
					maxv = v;
					maxi = i;
				}
			}
			ftr.filter_max_hist[maxi]++;

			// Lines
			const int line = linerow[c];
			if (line != -1) {
				if (!ftr.seen[line]) {
					ftr.seen[line] = 1;
					ftr.lines.push_back(line);
				}
				ftr.num_line_pix++;
				const int line_cla = vanpt_cla[vpts.owners[line]];
				if (line_cla == kVptVert) {
					ftr.pct_vert_vpt++;
					ftr.pct_vert_vpt_pix++;
				} else if (line_cla == kVptHoriz) {
					ftr.pct_horiz_vpt_pix++;
				}
			}
		}
	}

	//
	// Normalize over each segment
	//
	for (int i = 0; i < features.size(); i++) {
		HehFeature& ftr = features[i];
		float size = ftr.xs.size();

		// Shape
		ftr.area = size/image_size;

		sort_all(ftr.xs);
		sort_all(ftr.ys);

		ftr.x_mean = accumulate_all(ftr.xs, 0.0) / size;
		ftr.x_10pct = ftr.xs[size/10];
		ftr.x_90pct = ftr.xs[size*9/10];

		ftr.y_mean = accumulate_all(ftr.ys, 0.0) / size;
		ftr.y_10pct = ftr.ys[size/10];
		ftr.y_90pct = ftr.ys[size*9/10];

		ftr.y_mean_wrt_horizon = ftr.y_mean - horizon_y;
		ftr.y_10pct_wrt_horizon = ftr.y_10pct - horizon_y;
		ftr.y_90pct_wrt_horizon = ftr.y_90pct - horizon_y;

		// Color
		ftr.r_mean /= size;
		ftr.g_mean /= size;
		ftr.b_mean /= size;
		ftr.h_mean /= size;
		ftr.s_mean /= size;
		ftr.v_mean /= size;
		ftr.hue_hist /= size;
		ftr.sat_hist /= size;

		// Texture
		ftr.filter_means /= size;
		ftr.filter_max_hist /= size;

		// Lines
		int num_nearly_parallel = 0;
		BOOST_FOREACH(int i, ftr.lines) {
			BOOST_FOREACH(int j, ftr.lines) {
				if (i == j) continue;
				if (is_nearly_parallel[i][j]) {
					num_nearly_parallel++;
					ftr.pct_parallel_lines++;
					if (isct_is_right[i][j]) {
						ftr.pct_isct_right++;
					}
					if (isct_is_bottom[i][j]) {
						ftr.pct_isct_bottom++;
					}
					if (isct_bin[i][j] != -1) {
						ftr.isct_hist[isct_bin[i][j]]++;
					}
				}
			}
		}
		const int num_line_pairs = ftr.lines.size() * (ftr.lines.size()-1);
		const float sqrt_area = sqrt(size);  // don't use normalized area!
		ftr.pct_line_pix = ftr.num_line_pix / sqrt_area;
		if (num_line_pairs > 0) {
			ftr.pct_parallel_lines /= num_line_pairs;
		}
		if (num_nearly_parallel > 0) {
			ftr.isct_hist /= num_nearly_parallel;
			ftr.isct_hist_entropy = GetEntropy(ftr.isct_hist);
			ftr.pct_isct_right /= num_nearly_parallel;
			ftr.pct_isct_bottom /= num_nearly_parallel;
		}

		// Vanishing points
		bool has_horiz_vpt = false;
		bool has_vert_vpt = false;
		float horiz_vpt_x;
		float vert_vpt_y;
		for (int i = 0; i < ftr.lines.size(); i++) {
			const int owner_vpt = vpts.owners[ftr.lines[i]];
			if (vanpt_cla[owner_vpt] == kVptHoriz) {
				horiz_vpt_x = Project(norm_vpts[owner_vpt])[0];  // can be +/- INF
				has_horiz_vpt = true;
			}
			if (vanpt_cla[owner_vpt] == kVptHoriz) {
				vert_vpt_y = Project(norm_vpts[owner_vpt])[1];  // can be +/- INF
				has_vert_vpt = true;
			}
		}
		ftr.pct_vert_vpt_pix /= sqrt_area;
		ftr.pct_horiz_vpt_pix /= sqrt_area;
		if (ftr.num_line_pix > 0) {
			ftr.pct_vert_vpt /= ftr.num_line_pix;
		}
		if (has_horiz_vpt) {
			ftr.x_mean_wrt_hvpt = ftr.x_mean - horiz_vpt_x;  // can be +/- INF
			ftr.x_10pct_wrt_hvpt = ftr.x_10pct - horiz_vpt_x;  // can be +/- INF
			ftr.x_90pct_wrt_hvpt = ftr.x_90pct - horiz_vpt_x;  // can be +/- INF
		} else {
			ftr.x_mean_wrt_hvpt = 0;
			ftr.x_10pct_wrt_hvpt = 0;
			ftr.x_90pct_wrt_hvpt = 0;
		}
		if (has_vert_vpt) {
			ftr.y_mean_wrt_vert_vpt = ftr.y_mean - vert_vpt_y;
		} else {
			ftr.y_mean_wrt_vert_vpt = 0;
		}
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv, "../heh.cfg");

	// Read image
	ImageRGB<byte> imagergb;
	ImageF image;
	ReadImage(argv[1], imagergb);
	ImageCopy(imagergb, image);

	// Run segmentation
	MatI seg(image.GetHeight(), image.GetWidth());
	if (argc >= 3) {
		ReadMatrix(argv[2], seg);
	} else {
		FHSegmenter segmenter(image.GetWidth(), image.GetHeight());
		segmenter.Compute(image, seg, 1, 50, 150);
	}
	//DREPORT(seg);

	// Compute features
	HehFeatureGen gen;
	gen.Compute(imagergb, image, seg);

	for (int i = 0; i < gen.features.size(); i++) {
		DREPORT(i);
		SCOPED_INDENT;
		const HehFeature& f = gen.features[i];
		DLOG << f;
	}

	return 0;
}
