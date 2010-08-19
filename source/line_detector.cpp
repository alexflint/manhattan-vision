#include <iostream>
#include <iomanip>
#include <queue>

#include <boost/foreach.hpp>

#include <VW/Image/imagecopy.tpp>

#include <LU.h>

#include "common_types.h"
#include "image_bundle.h"
#include "canny.h"
#include "line_detector.h"
#include "geom_utils.h"

#include "eigensystem2d.tpp"
#include "math_utils.tpp"
#include "image_utils.tpp"

namespace indoor_context {
	using namespace toon;

	const lazyvar<float> gvMinCompSize("LineDetector.MinCompSize");
	const lazyvar<int> gvNumOrientBins("LineDetector.NumOrientBins");
	const lazyvar<int> gvOrientTol("LineDetector.OrientTol");

	static const PixelRGB<byte> kSpuriousColor(255, 255, 255);

	ostream& operator<< (ostream& s, const LineSeg& line) {
		s << "{LineSeg (" << line.start << ")->(" << line.end << ")}";
		return s;
	}

	void LineDetection::DrawPixels(ImageRGB<byte>& canvas,
																 const PixelRGB<byte>& color,
																 const toon::Vector<2> offs,
																 int thickness) const {
		BOOST_FOREACH(const ImageRef& p, *pixels) {
			DrawSpot(canvas, makeVector(p.x,p.y)+offs, color, thickness);
		}
		DrawSpot(canvas, project(seg.start)+offs, color, thickness+1);
		DrawSpot(canvas, project(seg.end)+offs, color, thickness+1);
	}

	void LineDetection::DrawLine(ImageRGB<byte>& canvas,
															 const PixelRGB<byte>& color,
															 const toon::Vector<2> offs,
															 int thickness) const {
		if (thickness==1) {
			indoor_context::DrawLine(canvas,
															 project(seg.start)+offs,
															 project(seg.end)+offs,
															 color);
		} else {
			DrawThickLineClipped(canvas,
													 project(seg.start)+offs,
													 project(seg.end)+offs,
													 color,
													 thickness);
		}
	}



	CannyLineDetector::CannyLineDetector() {
	}

	CannyLineDetector::CannyLineDetector(const ImageBundle& image) {
		Compute(image);
	}

	void CannyLineDetector::Compute(const ImageBundle& image) {
		input = &image;
		image.BuildMono();
		// Run canny edge detector
		canny.Compute(image.mono);
		// Find connected components
		FindComponents();
		// Fit lines
		FitLines();
	}

	void CannyLineDetector::FindComponents() {
		int nx = canny.edge_map.Cols();
		int ny = canny.edge_map.Rows();
		const int max_dim = max(nx, ny);
		const int min_size = max(max_dim * *gvMinCompSize, 2);

		// Prepare buffers
		detections.clear();
		seen.Resize(ny, nx);
		seen.Fill(0);
		segment_map.Resize(ny, nx);
		segment_map.Fill(-1);

		// Cache RingDist output for speed
		const int nbins = *gvNumOrientBins;
		MatI ringdist(nbins, nbins);
		for (int i = 0; i < nbins; i++) {
			for (int j = 0; j < nbins; j++) {
				ringdist[i][j] = RingDist<int>(i, j, nbins);
			}
		}

		// BFS from each segment
		const int tol = *gvOrientTol;
		LineDetection curseg;
		CHECK(comp_queue.empty()) << "comp_queue is not empty at start of line BFS";
		BOOST_FOREACH(const ImageRef& p, canny.edge_list) {
			const int dir = canny.dir16[p.y][p.x];
			const int* ringdistrow = ringdist[dir];
			if (seen[p.y][p.x] == 0) {
				comp_queue.push(p);
				seen[p.y][p.x] = 1;

				curseg.confidence = 0;
				curseg.pixels->clear();
			
				while (!comp_queue.empty()) {
					const ImageRef& p = comp_queue.front();
					curseg.pixels->push_back(p);
					curseg.confidence += sqrt(canny.magnitude_sqr[p.y][p.x]);
					comp_queue.pop();
				
					for (int yy = max(p.y-1, 0); yy <= min(p.y+1, ny-1); yy++) {
						for (int xx = max(p.x-1, 0); xx <= min(p.x+1, nx-1); xx++) {
							if (!(xx == p.x && yy == p.y) &&
									canny.edge_map[yy][xx] &&
									seen[yy][xx] == 0 &&
									ringdistrow[ canny.dir16[yy][xx] ] <= tol) {
								comp_queue.push(ImageRef(xx, yy));
								seen[yy][xx] = 1;
							}
						}
					}
				}
			}
		
			if (curseg.pixels->size() >= min_size) {
				BOOST_FOREACH(const ImageRef& p, *curseg.pixels) {
					segment_map[p.y][p.x] = detections.size();
				}
				detections.push_back(curseg);
				curseg.pixels.reset(new vector<ImageRef>);
			}
		}
	}

	void CannyLineDetector::FitLines() {
		//
		// Fit lines to each component
		//
		INDENTED BOOST_FOREACH(LineDetection& det, detections) {
			// Compute centroids
			Vector<2> mean = Zeros;
			BOOST_FOREACH(const ImageRef& p, *det.pixels) {
				mean[0] += p.x;
				mean[1] += p.y;
			}
			mean /= det.pixels->size();

			// Compute sums of normalized coords
			float sum_xx = 0, sum_xy = 0, sum_yy = 0;
			BOOST_FOREACH(const ImageRef& p, *det.pixels) {
				const float x = p.x - mean[0];
				const float y = p.y - mean[1];
				sum_xx += x*x;
				sum_xy += x*y;
				sum_yy += y*y;
			}

			// Compose the matrix and get eigenvals
			toon::Matrix<2> M;
			M[0][0] = sum_xx;
			M[0][1] = sum_xy;
			M[1][0] = sum_xy;
			M[1][1] = sum_yy;
			EigenSystem2D<double> decomp(M);

			// Compute line parameters
			det.confidence = decomp.eigval_large / decomp.eigval_small;
			Vector<2> direction = decomp.eigvec_large;

			// Determine line endpoints
			double projmin = INFINITY, projmax = -INFINITY;
			BOOST_FOREACH(const ImageRef& p, *det.pixels) {
				const toon::Vector<2> cur = makeVector(p.x, p.y);
				const double proj = (cur-mean) * direction;
				if (proj < projmin) {
					det.seg.start = unproject(cur);
					projmin = proj;
				}
				if (proj > projmax) {
					det.seg.end = unproject(cur);
					projmax = proj;
				}
			}

			// Compute line equation
			det.eqn = unit(det.seg.start ^ det.seg.end);
		}
	}

	void CannyLineDetector::Draw(ImageRGB<byte>& canvas) const {
		Draw(canvas, VecI(0));
	}

	void CannyLineDetector::Draw(ImageRGB<byte>& canvas,
															 const VecI& grouping,
															 const toon::Vector<2> offs) const {
		// Determine colors for the segments
		vector<PixelRGB<byte> > colors;
		for (int i = 0; i < detections.size(); i++) {
			if (grouping.size() == detections.size()) {
				if (grouping[i] < 0) {
					colors.push_back(kSpuriousColor);
				} else {
					colors.push_back(BrightColors::Get(grouping[i]%256));
				}
			} else {
				colors.push_back(RandomColor());
			}
		}
		// Draw the line segments
		Draw(canvas, colors, offs);
	}

	void CannyLineDetector::Draw(ImageRGB<byte>& canvas,
															 const vector<PixelRGB<byte> > colors,
															 const toon::Vector<2> offs) const {
		for (int i = 0; i < detections.size(); i++) {
			detections[i].DrawPixels(canvas, colors[i], offs);
		}
	}

	void CannyLineDetector::OutputLineViz(const string& filename) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		ResetAlpha(canvas);
		Draw(canvas);
		WriteImage(filename, canvas);
	}
}
