#include <iostream>
#include <iomanip>
#include <queue>

#include <boost/foreach.hpp>

#include <VW/Improc/smooth.h>
#include <VW/Image/imagecopy.tpp>

#include "common_types.h"
#include "fast_sobel.h"
#include "canny.h"
#include "filters.h"
//#include "numeric_utils.tpp"
#include "image_utils.tpp"

namespace indoor_context {
const lazyvar<float> gvSmoothingSigma("Gradients.SmoothingSigma");
const lazyvar<float> gvThreshLow("Canny.ThreshLow");
const lazyvar<float> gvThreshHigh("Canny.ThreshHigh");
const lazyvar<float> gvMinCompSize("Canny.MinCompSize");
const lazyvar<int> gvNumOrientBins("Canny.NumOrientBins");
const lazyvar<int> gvNumThreads("Canny.NumThreads");

// Clamp the position (r,c) to the matrix bounds and return the value
// at that point.
template <typename T>
inline T GetClamped(const VNL::Matrix<T>& mat, const int& r, const int& c) {
	return mat[Clamp(r, 0, mat.Rows()-1)][Clamp(c, 0, mat.Cols()-1)];
}

// Upsample an image by a factor of k
void Upsample(const MatF& input, MatF& output, int k) {
	assert(output.Cols() == input.Cols()*k);
	assert(output.Rows() == input.Rows()*k);
	for (int r = 0; r < output.Rows(); r++) {
		const float* inrow1 = input[r/k];
		const float* inrow2 = (r >= output.Rows()-k) ? inrow1 : input[r/k+1];
		float* outrow = output[r];
		for (int c = 0; c < output.Cols()-k; c++) {
			const int t = c%k;
			const int u = r%k;
			const float A = (k-t) * (k-u) * inrow1[c/k];
			const float B = t * (k-u) * inrow1[c/k+1];
			const float C = (k-t) * u * inrow2[c/k];
			const float D = t * u * inrow2[c/k+1];
			outrow[c] = (A+B+C+D) / (k*k);
		}
		// Deal with the last K columns seperately to avoid reading past
		// the end of rows
		for (int c = output.Cols()-k; c < output.Cols(); c++) {
			const int u = r%k;
			const float A = (k-u) * inrow1[c/k];
			const float C = u * inrow2[c/k];
			outrow[c] = (A+C) / k;
		}
	}
}


void Gradients::ComputeMagSqrRow(int r) {
	const int& w = diffx.GetWidth();
	const PixelMono<float>* dxrow = diffx[r];
	const PixelMono<float>* dyrow = diffy[r];
	float* out_magsqr = magnitude_sqr[r];
	for (int c = 0; c < w; c++) {
		const float& dx = dxrow[c].y;
		const float& dy = dyrow[c].y;
		out_magsqr[c] = dx*dx + dy*dy;
	}
}

void Gradients::ComputeOrientRow(int r) {
	const float kNbinsOverPi = *gvNumOrientBins / M_PI;
	const float kFourOverPi = 4.0 / M_PI;
	const int& w = diffx.GetWidth();
	const float thresh = atan_thresh * atan_thresh;
	const PixelMono<float>* dxrow = diffx[r];
	const PixelMono<float>* dyrow = diffy[r];
	const float* magsqr = magnitude_sqr[r];
	float* out_orient = orient[r];
	int* out_dir4 = dir4[r];
	int* out_dir16 = dir16[r];
	for (int c = 0; c < w; c++) {
		const float& dx = dxrow[c].y;
		const float& dy = dyrow[c].y;
		if (magsqr[c] >= thresh) {
			// atan2f is costly so only evaluate it if we really have to...
			out_orient[c] = atan2f(dy, dx);
			const float theta = out_orient[c] + M_PI;
			out_dir16[c] = static_cast<int>(roundf(theta*kNbinsOverPi)) % 16;
			out_dir4[c] = static_cast<int>(roundf(theta*kFourOverPi)) % 4;
		}
	}
}

void Gradients::ComputeMagSqrRows(const int r0, const int r1) {
	for (int r = r0; r <= r1; r++) {
		ComputeMagSqrRow(r);
	}
}

void Gradients::ComputeOrientRows(const int r0, const int r1) {
	for (int r = r0; r <= r1; r++) {
		ComputeOrientRow(r);
	}
}

void Gradients::ComputeMagSqr() {
	const int& w = diffx.GetWidth();
	const int& h = diffx.GetHeight();
	magnitude_sqr.Resize(h, w);
	ComputeMagSqrRows(0, h-1);
	// TODO: go back to the parallel version
	//ParallelPartition(h, *gvNumThreads, &mag_orient::ComputeMagSqrRows)
}

void Gradients::ComputeOrients() {
	const int& w = diffx.GetWidth();
	const int& h = diffy.GetHeight();

	orient.Resize(h, w);
	dir4.Resize(h, w);
	dir16.Resize(h, w);

	ComputeOrientRows(0, h-1);
	// TODO: go back to the parallel version
	//ParallelPartition(h, *gvNumThreads, &mag_orient::ComputeMagSqrRows)
}


void Gradients::ComputeSobel(const ImageF& rawinput) {
	// Smooth the image
	scoped_ptr<ImageF> smoothed;
	if (*gvSmoothingSigma > 0.0) {
		smoothed.reset(new ImageF);
		ImageCopy(rawinput, *smoothed);
		VW::SmoothUniform(*smoothed, ceili(*gvSmoothingSigma)*2+1);
	}
	const ImageF& input = *gvSmoothingSigma > 0 ? *smoothed : rawinput;

	// Run sobel filters
	ResizeImage(diffx, rawinput.GetSize());
	ResizeImage(diffy, rawinput.GetSize());
	FastSobel::ConvolveX(input, diffx);
	FastSobel::ConvolveY(input, diffy);
}

void Gradients::Compute(const ImageF& rawinput) {
	prev_input = &rawinput;
	ComputeSobel(rawinput);
	ComputeMagSqr();
	ComputeOrients();
}

void Gradients::Compute(const ImageBundle& input) {
	input.BuildMono();
	Compute(input.mono);
}

void CannyBase::NonMaxSuppression(MatF& gradient_mag_sqr,
                                  const MatI& gradient_dir4,
                                  vector<ImageRef>& high_list) {
	const int& w = gradient_mag_sqr.Cols();
	const int& h = gradient_mag_sqr.Rows();
	const float kThreshHighSqr = *gvThreshHigh * *gvThreshHigh;
	const float kThreshLowSqr = *gvThreshLow * *gvThreshLow;

	high_list.reserve(5000);
	for (int r = 0; r < h; r++) {
		float* magrow = gradient_mag_sqr[r];
		float* prevmagrow = gradient_mag_sqr[ (r==0 ? r : r-1) ];
		float* nextmagrow = gradient_mag_sqr[ (r==h-1 ? r : r+1) ];
		float* localrows[3] = {prevmagrow, magrow, nextmagrow};
		const int* dirrow = gradient_dir4[r];
		for (int c = 0; c < w; c++) {
			float& v = magrow[c];
			if (v >= kThreshLowSqr) {

				int dr, dc;
				switch (dirrow[c]) {
				case 0: dr = 0; dc = 1; break;  // north-south edge
				case 1: dr = 1; dc = 1; break; // southwest-northeast edge
				case 2: dr = 1; dc = 0; break;  // east-west edge
				case 3: dr = 1; dc = -1; break;  // southeast-northwest edge
				default:
					DLOG << "Error: dir=" << dirrow[c] << endl;
					exit(-1);
					break;
				}

				const int nextc = Clamp(c+dc, 0, w-1);
				const int prevc = Clamp(c-dc, 0, w-1);
				// Must use abs() here because we do v=-v below
				if (v < abs(localrows[dr+1][nextc]) ||
						v < abs(localrows[-dr+1][prevc])) {
					// We must not set v=0 because other pixels might be
					// affected by this pixel. Instead set v=-v and use abs()
					// above, then do another pass over the image below
					v = -v;
				} else if (v >= kThreshHighSqr) {
					high_list.push_back(ImageRef(c, r));
				}
			}
		}
	}

	// Set the negative values to zero
	for (int r = 0; r < h; r++) {
		float* row = gradient_mag_sqr[r];
		for (int c = 0; c < w; c++) {
			if (row[c] < 0) {
				row[c] = 0;
			}
		}
	}
}

void CannyBase::Hysterisis(const MatF& gradient_mag_sqr,
                           const vector<ImageRef>& high_list) {
	const int& w = gradient_mag_sqr.Cols();
	const int& h = gradient_mag_sqr.Rows();
	const float kThreshLowSqr = *gvThreshLow * *gvThreshLow;

	// Initialize the output containers
	edge_list.clear();
	edge_list.reserve(high_list.size() / 4);
	edge_map.Resize(h, w);
	edge_map.Fill(0);
	queue<ImageRef> queue;  // re-use for efficiency

	// Iterate over the pixels that are above the high threshold
	BOOST_FOREACH(const ImageRef& p, high_list) {
		// Check that we haven't already visited here
		if (!edge_map[p.y][p.x]) {
			queue.push(p);
			edge_map[p.y][p.x] = 1;

			// BFS from here
			while (!queue.empty()) {
				const int r = queue.front().y;
				const int c = queue.front().x;
				edge_list.push_back(queue.front());
				queue.pop();

				for (int nr = r-1; nr <= r+1; nr++) {
					for (int nc = c-1; nc <= c+1; nc++) {
						if (nr == r && nc == c) continue;
						if (nr<0 || nr>=h || nc<0 || nc>=w) continue;
						if (!edge_map[nr][nc] &&
								gradient_mag_sqr[nr][nc] > kThreshLowSqr) {
							queue.push(ImageRef(nc, nr));
							edge_map[nr][nc] = 1;
						}
					}
				}
			}
		}
	}
}

void CannyBase::DetectEdges(MatF& gradient_mag_sqr,
                            const MatI& gradient_dir4) {
	vector<ImageRef> high_list;
	NonMaxSuppression(gradient_mag_sqr, gradient_dir4, high_list);
	Hysterisis(gradient_mag_sqr, high_list);
}

void Canny::Compute(const ImageBundle& input) {
	input.BuildMono();
	Compute(input.mono);
}

void Canny::Compute(const ImageF& input) {
	// Compute gradient magnitude and orientation
	gradients.atan_thresh = *gvThreshLow;
	gradients.Compute(input);
	// Evaluate the detector
	DetectEdges(gradients.magnitude_sqr, gradients.dir4);
}

void MultiScaleCanny::Compute(const ImageF& input) {
	const int& w = input.GetWidth();
	const int& h = input.GetHeight();
	const float kThreshLowSqr = *gvThreshLow * *gvThreshLow;
	dir4.Resize(h, w);
	dir16.Resize(h, w);
	magnitude_sqr.Resize(h, w);
	magnitude_sqr.Fill(0);

	// Compute gradients at multiple scales
	MatF mag_sqr_upsampled;
	const ImageF* cur_level = &input;
	const MatF* cur_mag_sqr;
	scoped_ptr<ImageF> storage;
	for (int i = 0; i < gradients.size(); i++) {
		// Compute gradients
		gradients[i]->atan_thresh = kThreshLowSqr;
		gradients[i]->Compute(*cur_level);

		// Upsample
		if (i == 0) {
			cur_mag_sqr = &gradients[i]->magnitude_sqr;
		} else {
			mag_sqr_upsampled.Resize(h, w);
			Upsample(gradients[i]->magnitude_sqr, mag_sqr_upsampled, 1<<i);
			cur_mag_sqr = &mag_sqr_upsampled;
		}

		// Take maximum
		for (int r = 0; r < h; r++) {
			const float* in_mag = (*cur_mag_sqr)[r];
			float* out_mag = magnitude_sqr[r];
			for (int c = 0; c < w; c++) {
				out_mag[c] = max(in_mag[c], out_mag[c]);
			}
		}

		// Downsample
		if (i < gradients.size()-1) {
			CHECK(false)
				<< "This part needs to be fixed: gradients.smoothed no longer exists";
		//storage.reset(Downsample(gradients[i]->smoothed, 2));
			cur_level = storage.get();
		}
	}

	// Apply non-maximal suppression and hysterisis
	dir4 = gradients[0]->dir4;
	dir16 = gradients[0]->dir16;
	DetectEdges(magnitude_sqr, dir4);
}

}
