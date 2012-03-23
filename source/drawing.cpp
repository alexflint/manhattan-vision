#include "drawing.h"

#include <boost/foreach.hpp>

#include <TooN/so3.h>

#include "common_types.h"
#include "camera.h"
#include "line_segment.h"
#include "line_detector.h"
#include "geom_utils.h"
#include "vpt_utils.h"
#include "lazyvar.h"

#include "canvas.h"
#include "colors.h"

namespace indoor_context {
	using namespace toon;

	lazyvar<bool> gvFatLines("VptVizualization.FatLines");

	void DrawSegmentWithExtension(const LineSeg& seg,
																const PosedCamera& pc,
																const SO3<>& R,
																int axis,
																const PixelRGB<byte>& color,
																Canvas& canvas) {
		double kPointSize = *gvFatLines ? 6. : 2.;
		double kSegmentWidth = *gvFatLines ? 4. : 1.5;
		double kExtensionWidth = *gvFatLines ? 1.5 : .5;
		Vec3 vpt = ProjectVanishingPoint(axis, R, pc);
		Vec2 a = project(seg.start);
		Vec2 b = project(seg.end);
		Vec2 m = project(seg.midpoint());
		Vec2 v = project(vpt);
		canvas.SetLineWidth(kExtensionWidth);
		canvas.StrokeLine(m, v, color);
		canvas.SetLineWidth(kSegmentWidth);
		canvas.StrokeLine(a, b, color);
		canvas.DrawDot(a, kPointSize, color);
		canvas.DrawDot(b, kPointSize, color);
	}

	void OutputVptViz(const PosedImage& image,
										const vector<LineDetection>& detections,
										const SO3<>& R,
										const string& path) {
		vector<LineSeg> segments;
		BOOST_FOREACH (const LineDetection& det, detections) {
			segments.push_back(det.seg);
		}
		OutputVptViz(image, segments, R, path);
	}

	void OutputVptViz(const PosedImage& image,
										const vector<LineSeg>& segments,
										const SO3<>& R,
										const string& path) {
		vector<int> axes;
		BOOST_FOREACH(const LineSeg& segment, segments) {
			axes.push_back(ComputeMostLikelyAxis(image.pc(), R, segment));
		}
		OutputVptViz(image, segments, axes, R, path);
	}

	void OutputVptViz(const PosedImage& image,
										const vector<LineSeg>& segments,
										const vector<int>& axes,
										const toon::SO3<>& R,
										const string& path) {
		CHECK_EQ(segments.size(), axes.size());

		// Copy the original image into the vizualization
		Vec2I pad = makeVector(100, 100);
		Vec2I size = image.size() + pad*2;
		FileCanvas canvas(path, size);

		// Note that DrawImage() applies a transformation to the canvas so
		// that (0,0) is at the top left of the image. This means we don't
		// have to worry about the padding from here on.
		canvas.DrawImage(image.rgb, 1., pad);

		// Draw the lines and extensions
		for (int i = 0; i < segments.size(); i++) {
			PixelRGB<byte> color = Colors::primary(axes[i]);
			DrawSegmentWithExtension(segments[i], image.pc(), R, axes[i], color, canvas);
		}

		// Draw the vanishing points
		for (int i = 0; i < 3; i++) {
			Vec3 vpt = ProjectVanishingPoint(i, R, image.pc());
			if (abs(vpt[2]) > 1e-8) {
				Vec2 p = project(vpt);
				canvas.DrawDot(p, 4., Colors::white());
				canvas.DrawDot(p, 3., Colors::primary(i));
			}
		}
	}
}
