#include <boost/ptr_container/ptr_vector.hpp>

#include <VW/Image/imagecopy.tpp>

#include "line_sweeper.h"
#include "common_types.h"
#include "guided_line_detector.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
#include "fill_polygon.tpp"

namespace indoor_context {
using namespace toon;

lazyvar<int> gvBlockMarginSqr("LineSweeper.BlockMarginSqr");

LineSweeper::LineSweeper() : input(NULL) {
}

LineSweeper::LineSweeper(const PosedImage& image,
                         const vector<LineDetection> lines[]) {
	Compute(image, lines);
}

void LineSweeper::Compute(const PosedImage& image,
                          const vector<LineDetection> lines[]) {
	input = &image;

	// Get bounding lines for the image
	Bounds2D<double> image_bounds = Bounds2D<double>::FromSize(image.sz());
	vector<Vec3 > bounding_lines;
	bounding_lines.push_back(image_bounds.left_eqn());      // left
	bounding_lines.push_back(image_bounds.top_eqn());      // top
	bounding_lines.push_back(image_bounds.right_eqn());      // right
	bounding_lines.push_back(image_bounds.bottom_eqn());      // bottom

	// Build support maps for each direction
	int nan_sweeps = 0;
	vector<Vec3 > blocks;
	blocks.reserve(lines[0].size() + lines[1].size() + lines[2].size()); // upper bound
	for (int i = 0; i < 3; i++) {
		support_maps[i].Resize(image.ny(), image.nx());
		support_maps[i].Fill(0);
		num_pixels[i] = 0;

		for (int j = 0; j < 3; j++) {
			if (i == j) continue;
			const Vec3 pivot = image.pc().GetImageVpt(i);
			const Vec3 dir = image.pc().GetImageVpt(j);

			// TODO: Set these to projection with image bounds:
			BOOST_FOREACH(const LineDetection& det, lines[i]) {
				// Compute lines from the endpoints of the current segment to
				// the j-th vanishing point
				const Vec3 vline1 = det.seg.start ^ dir;
				const Vec3 vline2 = det.seg.end ^ dir;
				const int sign1 = HSignDP(det.seg.end, vline1);
				const int sign2 = HSignDP(det.seg.start, vline2);

				// Generate a list of potentially-blocking points
				blocks.clear();

				// Add intersections with image bounds
				BOOST_FOREACH(const Vec3& bound, bounding_lines) {
					// don't want one part of the image boundary to block the
					// region before the edge of the image, so only add the most
					// distant intersection
					Vec3 isct1 = vline1 ^ bound;
					Vec3 isct2 = vline2 ^ bound;
					double proj1 = (det.eqn * isct1) / isct1[2];
					double proj2 = (det.eqn * isct2) / isct2[2];
					if (abs(proj1) >= abs(proj2)) {
						blocks.push_back(isct1);
					} else {
						blocks.push_back(isct2);
					}
				}

				// Add the vanishing point itself as a potential block
				blocks.push_back(dir);

				// Add all line segments associated with opposing vanishing points
				int opp = (2*i-j+3)%3;  // opp is the axis that is not i or j
				BOOST_FOREACH(const LineDetection& block, lines[opp]) {
					// Get endpoints of the potentially-blocking line segment
					// do not use references here, we are going to _modify_
					// these in the ClipAgainstLine calls
					Vec3 a = block.seg.start;  // will be _modified_ later
					Vec3 b = block.seg.end;  // will be _modified_ later

					// Clip the potentially-blocking line segment
					bool clip1 = ClipAgainstLine(a, b, vline1, sign1);
					bool clip2 = ClipAgainstLine(a, b, vline2, sign2);
					if (clip1 && clip2) {
						blocks.push_back(a);
						blocks.push_back(b);
					} else if (*gvBlockMarginSqr > 0) {
						// The line segment lies entirely outside the clip area,
						// but still include it as a blocker if it is within a
						// threshold distance. Note that "a" is now the
						// intersection point due to the call to ClipAgainstLine.
						double dist1 = norm_sq(project(a) - project(block.seg.start));
						double dist2 = norm_sq(project(a) - project(block.seg.end));
						if (min(dist1, dist2) < *gvBlockMarginSqr) {
							blocks.push_back(a);
						}
					}
				}

				// Find the closest block infront and behind
				Vec3 front_line, back_line;
				double front_proj = INFINITY, back_proj = -INFINITY;
				BOOST_FOREACH(const Vec3& block, blocks) {
					// Get intersection with the sweep line
					const Vec3 pivot_line = block ^ pivot;
					const Vec3 proj_pt = pivot_line ^ vline1;

					// Shouldn't have any pts-at-inf by now...
					const double proj = (det.eqn * proj_pt) / proj_pt[2];
					if (proj > 0 && proj < front_proj) {
						front_proj = proj;
						front_line = pivot_line;
					} else if (proj < 0 && proj > back_proj) {
						back_proj = proj;
						back_line = pivot_line;
					}
				}

				// Compute the polygon vertices
				vector<Vec3> poly;
				poly.push_back(front_line ^ vline1);
				poly.push_back(front_line ^ vline2);
				poly.push_back(back_line ^ vline2);
				poly.push_back(back_line ^ vline1);

				bool found_nan = false;
				BOOST_FOREACH(const Vec3 v, poly) {
					if (isnan(v)) {
						found_nan = true;
						nan_sweeps++;
						break;
					}
				}

				// Mark the area
				if (!found_nan) {
					FillPolygon(poly, support_maps[i], 1);
				}
			}
		}
	}

	if (nan_sweeps > 0) {
		DLOG << "Warning: Ignored " << nan_sweeps << " sweeps due to NaN coordinates during line sweep";
	}
}

void LineSweeper::OutputSupportViz(const string& basename) const {
	CHECK(input);
	for (int i = 0; i < 3; i++) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		DrawStencil(support_maps[i], canvas, BrightColors::Get(i, 0.5));
		WriteImage(basename+itoa(i)+".png", canvas);
	}
}



void GeomLabellerBase::Init(const PosedImage& image) {
	input = &image;
	orient_map.Resize(image.ny(), image.nx(), 0);
}

void GeomLabellerBase::DrawOrientViz(ImageRGB<byte>& canvas) const {
	CHECK(input);
	ResizeImage(canvas, input->sz());
	for (int y = 0; y < input->ny(); y++) {
		const int* labelrow = orient_map[y];
		PixelRGB<byte>* outrow = canvas[y];
		for (int x = 0; x < input->nx(); x++) {
			if (labelrow[x] >= 0) {
				BlendWith(outrow[x], Colors::primary(labelrow[x]), 0.5);
			}
		}
	}
}



IsctGeomLabeller::IsctGeomLabeller() {
}

IsctGeomLabeller::IsctGeomLabeller(const PosedImage& image) {
	GuidedLineDetector line_detector(image);
	Compute(image, line_detector.detections);
}

IsctGeomLabeller::IsctGeomLabeller(const PosedImage& image,
                                   const vector<LineDetection> lines[]) {
	Compute(image, lines);
}

void IsctGeomLabeller::Compute(const PosedImage& image,
                               const vector<LineDetection> lines[]) {
	Init(image);

	// Compute line sweeps
	sweeper.Compute(image, lines);

	// Produce final label maps
	for (int i = 0; i < 3; i++) {
		int sumi = 0;
		for (int y = 0; y < image.ny(); y++) {
			const int* suprow = sweeper.support_maps[i][y];
			int* orientrow = orient_map[y];
			for (int x = 0; x < image.nx(); x++) {
				orientrow[x] |= suprow[x]<<i;
			}
		}
	}

	for (int y = 0; y < image.ny(); y++) {
		int* orientrow = orient_map[y];
		for (int x = 0; x < image.nx(); x++) {
			// the numbers below are chosen to map:
			//   011 -> 2
			//   101 -> 1
			//   110 -> 0
			// these choices are important as they agree with LeeRecovery conventions
			switch (orientrow[x]) {
			case 3: orientrow[x] = 2; break;
			case 5: orientrow[x] = 1; break;
			case 6: orientrow[x] = 0; break;
			default: orientrow[x] = -1; break;
			}
		}
	}
}

void IsctGeomLabeller::OutputOrientViz(const string& filename) const {
	CHECK_NOT_NULL(input);
	ImageRGB<byte> canvas;
	ImageCopy(input->rgb, canvas);
	DrawOrientViz(canvas);
	WriteImage(filename, canvas);
}

}
