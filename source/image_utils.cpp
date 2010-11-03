#include <boost/bind.hpp>

#include <helpers.h>

#include "common_types.h"
#include "clipping.h"

#include "image_utils.tpp"
#include "range_utils.tpp"
//#include "numeric_utils.tpp"
#include "vector_utils.tpp"
#include "polygon.tpp"

namespace indoor_context {
using namespace toon;

void DrawSegmentation(const MatI& seg, ImageRGB<byte>& output) {
	BrightColors gen;
	vector<PixelRGB<byte> > colors;
	generate_n(back_inserter(colors),
	           seg.MaxValue()+1,
	           bind(&BrightColors::Next, ref(gen)));
	DrawSegmentation(seg, colors, output);
}

void DrawSegmentation(const MatI& mat,
                      const vector<PixelRGB<byte> >& colors,
                      ImageRGB<byte>& output) {
	ResizeImage(output, mat.Cols(), mat.Rows());
	for (int r = 0; r < mat.Rows(); r++) {
		const int* in = mat[r];
		PixelRGB<byte>* out = output[r];
		for (int c = 0; c < mat.Cols(); c++) {
			out[c] = colors[in[c]];
		}
	}
}

void DrawSpot(ImageRGB<byte>& image,
              const Vec2& p,
              const PixelRGB<byte>& color,
              const int size) {
	const int cx = roundi(p[0]);
	const int cy = roundi(p[1]);
	const int x1 = max(cx-size, 0);
	const int x2 = min(cx+size+1, image.GetWidth());
	const int y1 = max(cy-size, 0);
	const int y2 = min(cy+size+1, image.GetHeight());
	for (int y = y1; y < y2; y++) {
		PixelRGB<byte>* row = image[y];
		for (int x = x1; x < x2; x++) {
			row[x] = color;
		}
	}
}

void DrawThickLineClipped(ImageRGB<byte>& canvas,
                          const Vec2& start,
                          const Vec2& end,
                          const PixelRGB<byte>& color,
                          const int w) {
	for (int dy = -w; dy <= w; dy++) {
		for (int dx = -w; dx <= w; dx++) {
			Vec2 d = makeVector(dx, dy);
			DrawLineClipped(canvas, start+d, end+d, color);
		}
	}
}


void DrawLineClipped(ImageRGB<byte>& image,
                     const float x0,
                     const float y0,
                     const float x1,
                     const float y1,
                     const PixelRGB<byte> color,
                     double alpha) {
	Vec3 a = makeVector(x0, y0, 1.0);
	Vec3 b = makeVector(x1, y1, 1.0);
	bool nonempty = ClipAgainstBounds(a, b, Bounds2D<>::FromTightSize(asToon(image.GetSize())));
	if (nonempty) {
		DrawLine(image, project(a), project(b), color, alpha);
	}
}

void DrawLineClipped(ImageRGB<byte>& image,
                     const Vec2& a,
                     const Vec2& b,
                     const PixelRGB<byte>& color,
                     double alpha) {
	DrawLineClipped(image, a[0], a[1], b[0], b[1], color, alpha);
}

void DrawLineClipped(ImageRGB<byte>& image,
                     const LineSeg& seg,
                     const PixelRGB<byte>& color,
                     double alpha) {
	DrawLineClipped(image, project(seg.start), project(seg.end), color, alpha);
}

void DrawLine(ImageRGB<byte>& canvas,
              const Vec2& start,
              const Vec2& end,
              const PixelRGB<byte>& color,
              const float alpha) {
	double length = norm(end-start);
	Vec2 step = unit(end-start);
	Vec2 cur = start;
	for (int i = 0; i <= length; i++) {
		const int x = roundi(cur[0]);
		const int y = roundi(cur[1]);
		if (x >= 0 && y >= 0 && x < canvas.GetWidth() && y < canvas.GetHeight()) {
			BlendWith(canvas[y][x], color, 1.0-alpha);
		}
		cur += step;
	}
}

void DrawLine(ImageRGB<byte>& canvas,
              const LineSeg& seg,
              const PixelRGB<byte>& color) {
	DrawLine(canvas, project(seg.start), project(seg.end), color);
}

void DrawStencil(const MatI& stencil,
                 ImageRGB<byte>& canvas,
                 const PixelRGB<byte>& color) {
	for (int y = 0; y < Height(stencil); y++) {
		const int* strow = stencil[y];
		PixelRGB<byte>* crow = canvas[y];
		for (int x = 0; x < Width(stencil); x++) {
			if (strow[x]) {
				BlendWith(crow[x], color, 1.0*color.alpha/255.0);
			}
		}
	}
}

void DrawOrientations(const MatI& orients,
                      ImageRGB<byte>& canvas,
                      double alpha) {
	if (alpha == 1.0) {
		ResizeImage(canvas, orients.Cols(), orients.Rows());
	}
	PixelRGB<byte> color;
	for (int y = 0; y < orients.Rows(); y++) {
		PixelRGB<byte>* outrow = canvas[y];
		const int* inrow = orients[y];
		for (int x = 0; x < orients.Cols(); x++) {
			const PixelRGB<byte>& c = inrow[x] == -1 ? Colors::white() : Colors::primary(inrow[x]);
			if (alpha == 1.0) {
				outrow[x] = c;
			} else if (inrow[x] != -1) {
				BlendWith(outrow[x], c, 1-alpha);
			}
		}
	}
}

void DrawOrientations(const MatI& orients, ImageRGB<byte>& canvas, const ImageRef& sz) {
	ResizeImage(canvas, sz);
	for (int y = 0; y < sz.y; y++) {
		PixelRGB<byte>* row = canvas[y];
		const int* orient_row = orients[y*orients.Rows()/sz.y];
		for (int x = 0; x < sz.x; x++) {
			int orient = orient_row[x*orients.Cols()/sz.x];
			if (orient == -1) {
				row[x] = PixelRGB<byte>(255,255,255);
			} else {
				row[x] = Colors::primary(orient);
			}
		}
	}
}

void WriteOrientationImage(const string& filename, const MatI& orients) {
	WriteOrientationImage(filename, orients, ImageRef(orients.Cols(), orients.Rows()));
}

void WriteOrientationImage(const string& filename,
                           const MatI& orients,
                           const ImageRef& sz) {
	ImageRGB<byte> canvas;
	DrawOrientations(orients, canvas, sz);
	WriteImage(filename, canvas);
}

PixelRGB<byte> Blend(const PixelRGB<byte>& p1,
		const PixelRGB<byte>& p2,
		const float alpha) {
	PixelRGB<byte> out = p1;
	BlendWith(out, p2, alpha);
	return out;
}

void BlendWith(PixelRGB<byte>& p1,
               const PixelRGB<byte>& p2,
               const float a) {
	p1.r = static_cast<byte>(a*p1.r + (1.0-a)*p2.r);
	p1.g = static_cast<byte>(a*p1.g + (1.0-a)*p2.g);
	p1.b = static_cast<byte>(a*p1.b + (1.0-a)*p2.b);
}

void ResetAlpha(ImageRGB<byte>& image) {
	for (int r = 0; r < image.GetHeight(); r++) {
		PixelRGB<byte>* row = image[r];
		for (int c = 0; c < image.GetWidth(); c++) {
			row[c].alpha = 0;
		}
	}
}

void DrawHistogram(ImageRGB<byte>& canvas,
                   const vector<int>& hist,
                   float aspect) {
	int nx = hist.size();
	int ny = roundi(aspect*nx);
	ResizeImage(canvas, nx, ny);
	int maxv = *max_element_n(&hist[0], hist.size());
	for (int y = 0; y < ny; y++) {
		int cutoff = (ny-y)*maxv/ny;
		PixelRGB<byte>* row = canvas[y];
		for (int x = 0; x < nx; x++) {
			row[x] = (hist[x] >= cutoff) ? Colors::black() : Colors::white();
		}
	}
}

void CopyImageScaled(const ImageRGB<byte>& image,
                     int destx,
                     int desty,
                     int destsize,
                     ImageRGB<byte>& canvas) {
	const int srcw = image.GetWidth();
	const int srch = image.GetHeight();
	int destw, desth;
	if (srcw > srch) {
		destw = destsize;
		desth = srch * destw / srcw;
		desty += (destsize - desth) / 2;
	} else {
		desth = destsize;
		destw = srcw * desth / srch;
		destx += (destsize - destw) / 2;
	}

	for (int y = 0; y < desth; y++) {
		const int srcy = y * srch / desth;
		const PixelRGB<byte>* srcrow = image[srcy];
		PixelRGB<byte>* destrow = &canvas[desty+y][destx];
		for (int x = 0; x < destw; x++) {
			const int srcx = x * srcw / destw;
			destrow[x] = srcrow[srcx];
		}
	}
}

void CopyImageInto(const ImageRGB<byte>& src,
                   int destr,
                   int destc,
                   ImageRGB<byte>& canvas) {
	for (int r = 0; r < src.GetHeight(); r++) {
		const PixelRGB<byte>* inrow = src[r];
		PixelRGB<byte>* outrow = canvas[destr+r]+destc;
		for (int c = 0; c < src.GetWidth(); c++) {
			outrow[c] = inrow[c];
		}
	}
}

int ImageToSegmentation(const ImageRGB<byte>& image, MatI& seg, const int first) {
	int num_regions = 0;
	map<int, int> region_ids;
	for (int r = 0; r < image.GetHeight(); r++) {
		const PixelRGB<byte>* imagerow = image[r];
		int* segrow = seg[r];
		for (int c = 0; c < image.GetWidth(); c++) {
			const int code = (imagerow[c].r<<16) | (imagerow[c].g<<8) | (imagerow[c].b);
			map<int, int>::iterator it = region_ids.find(code);
			if (it == region_ids.end()) {
				region_ids[code] = first + num_regions++;
			}
			segrow[c] = region_ids[code];
		}
	}
	return num_regions;
}
}
