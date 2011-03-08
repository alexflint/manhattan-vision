#include "canvas.h"

#include <boost/filesystem.hpp>

#include <cairomm/cairomm.h>
#include <cairomm/context.h>
#include <cairomm/surface.h>

#include "common_types.h"
#include "image_utils.h"

#include "canvas.tpp"
#include "polygon.tpp"
#include "vector_utils.tpp"
#include "vw_image.tpp"

namespace indoor_context {
using namespace toon;
using namespace Cairo;

const double Canvas::kDefaultDashLength = 3.0;

void Canvas::PushState() {
	c_->save();
}

void Canvas::Translate(const Vec2& t) {
	c_->translate(t[0], t[1]);
}

void Canvas::MoveTo(const Vec2& p) {
	c_->move_to(p[0], p[1]);
}

void Canvas::MoveTo(const Vec3& p) {
	MoveTo(project(p));
}

void Canvas::LineTo(const Vec2& p) {
	c_->line_to(p[0], p[1]);
}

void Canvas::LineTo(const Vec3& p) {
	LineTo(project(p));
}

void Canvas::Stroke() {
	c_->stroke();
}

void Canvas::Scale(const Vec2& s) {
	c_->scale(s[0], s[1]);
}

void Canvas::Scale(double s) {
	c_->scale(s, s);
}

void Canvas::Transform(const Mat2& m, const Vec2& t) {
	// This ordering is documented at
	// http://cairographics.org/manual/cairo-matrix.html#cairo-matrix-init
	c_->transform(Cairo::Matrix(m[0][0], m[1][0], m[0][1], m[1][1], t[0], t[1]));
}

void Canvas::SetTransform(const Mat2& m, const Vec2& t) {
	// This ordering is documented at
	// http://cairographics.org/manual/cairo-matrix.html#cairo-matrix-init
	c_->set_matrix(Cairo::Matrix(m[0][0], m[1][0], m[0][1], m[1][1], t[0], t[1]));
}

void Canvas::PopState() {
	c_->restore();
}

void Canvas::SetColor(const PixelRGB<byte>& p) {
	// The VW convention for alpha is 0=opaque, 255=transparent but
	// the Cairo convention is 0=transparent, 1=opaque
	c_->set_source_rgba(1.0*p.r/255.0, 1.0*p.g/255.0, 1.0*p.b/255.0, 1.0-(1.0*p.alpha/255.0));
}

void Canvas::SetLineWidth(double v) {
	c_->set_line_width(v);
}

void Canvas::SetLineJoin(Cairo::LineJoin v) {
	c_->set_line_join(v);
}

void Canvas::SetLineCap(Cairo::LineCap v) {
	c_->set_line_cap(v);
}

void Canvas::SetLineDash(vector<double>& pattern) {
	c_->set_dash(pattern, 0.0);  // second param is the offset at which to start
}

void Canvas::SetLineDash(double length) {
	vector<double> pattern;
	// Cairo understands patterns of length 1 as dashes of that length
	pattern.push_back(length);
	SetLineDash(pattern);
}

void Canvas::SetLineDash(bool enabled) {
	vector<double> pattern;
	if (enabled) {
		pattern.push_back(kDefaultDashLength);
	}
	// Cairo understands patterns of length 0 as disable dashing
	SetLineDash(pattern);
}

void Canvas::DrawDot(const Vec2& c) {
	DrawDot(c, 1.0);
}

void Canvas::DrawDot(const Vec2& c, const PixelRGB<byte>& color) {
	SetColor(color);
	DrawDot(c);
}

void Canvas::DrawDot(const Vec2& c, double radius) {
	c_->arc(c[0], c[1], radius, 0.0, 2.0*M_PI);
	c_->fill();
}

void Canvas::DrawDot(const Vec2& c, double radius, const PixelRGB<byte>& color) {
	SetColor(color);
	DrawDot(c, radius);
}

void Canvas::StrokeLine(const Vec2& a, const Vec2& b) {
	c_->move_to(a[0], a[1]);
	c_->line_to(b[0], b[1]);
	c_->stroke();
}

void Canvas::StrokeLine(const Vec2& a, const Vec2& b, const PixelRGB<byte>& color) {
	SetColor(color);
	StrokeLine(a, b);
}

void Canvas::StrokeLine(const LineSeg& seg) {
	StrokeLine(project(seg.start), project(seg.end));
}

void Canvas::StrokeLine(const LineSeg& seg, const PixelRGB<byte>& color) {
	StrokeLine(project(seg.start), project(seg.end), color);
}

void Canvas::Clear(const PixelRGB<byte>& bg_color) {
	SetColor(bg_color);
	c_->fill();  // fill with no path or mask fills everything
}

void Canvas::DrawImageRescaled(const MatF& image, double alpha) {
	float scale = 255.0 / image.MaxValue();
	ImageRGB<byte> buffer(image.Cols(), image.Rows());
	for (int y = 0; y < image.Rows(); y++) {
		const float* in = image[y];
		PixelRGB<byte>* out = buffer[y];
		for (int x = 0; x < image.Cols(); x++) {
			out[x].r = out[x].g = out[x].b = 255*pow(scale*in[x]/255.0, 2.0);
		}
	}
	ResetAlpha(buffer);
	DrawImage(buffer);
}	

void Canvas::DrawImage(const ImageRGB<byte>& image, double alpha) {
	PushState();

	// Reformat the pixel data
	int bpp = 4;
	int nx = image.GetWidth();
	int ny = image.GetHeight();
	int stride = ImageSurface::format_stride_for_width(FORMAT_ARGB32, nx);
	scoped_array<byte> data(new byte[stride*ny]);
	for (int r = 0; r < ny; r++) {
		const PixelRGB<byte>* inrow = image[r];
		byte* outrow = data.get() + r*stride;
		for (int c = 0; c < nx; c++) {
			const PixelRGB<byte>& p = inrow[c];
			*reinterpret_cast<int*>(outrow) = ((255-p.alpha)<<24) | (p.r<<16) | (p.g<<8) | p.b;
			outrow += bpp;
		}
	}

	// Create a surface for the data
	RefPtr<ImageSurface> surf
	= ImageSurface::create(reinterpret_cast<byte*>(data.get()),
	                       FORMAT_ARGB32, nx, ny, stride);

	// Draw the surface
	PushState();
	c_->set_source(surf, 0, 0);  // the last two parameters are the x and y offset
	if (alpha == 1.0) {
		c_->paint();
	} else {
		c_->paint_with_alpha(alpha);
	}

	PopState();

	// Destroy the surface before the data goes out of scope
	surf->finish();
}

void Canvas::DrawImage(const ImageRGB<byte>& image,
                       double s,
                       const Vec2& t,
                       double alpha) {
	DrawImage(image, Identity*s, t, alpha);
}

void Canvas::DrawImage(const ImageRGB<byte>& image,
                       const Mat2& m,
                       const Vec2& t,
                       double alpha) {
	PushState();
	SetTransform(m, t);
	DrawImage(image, alpha);
	PopState();
}






FileCanvas::FileCanvas(const string& filename,
                       const Vec2I& size)
: filename_(filename), size_(size), written_(false) {
	format_ = GetFormat(filename);
	InitSurface();
}

FileCanvas::FileCanvas(const string& filename,
											 const Vec2I& size,
                       const PixelRGB<byte>& bg)
: filename_(filename), size_(size), written_(false) {
	format_ = GetFormat(filename);
	InitSurface();
	Clear(bg);
}

FileCanvas::FileCanvas(const string& filename,
                       const ImageRGB<byte>& bg)
: filename_(filename), size_(asToon(bg.GetSize())), written_(false) {
	format_ = GetFormat(filename);
	InitSurface();
	DrawImage(bg);
}

FileCanvas::FileCanvas(const string& filename,
                       const MatF& bg)
	: filename_(filename), size_(makeVector(bg.Cols(), bg.Rows())), written_(false) {
	format_ = GetFormat(filename);
	InitSurface();
	DrawImageRescaled(bg);
}

	/*
FileCanvas::FileCanvas(Format format,
                       const string& filename,
                       const Vec2I& size)
: filename_(filename), format_(format), size_(size), written_(false) {
	InitSurface();
}

FileCanvas::FileCanvas(Format format,
                       const string& filename,
                       const ImageRGB<byte>& bg)
: filename_(filename), format_(format), size_(asToon(bg.GetSize())), written_(false) {
	InitSurface();
	DrawImage(bg);
}

FileCanvas::FileCanvas(Format format,
                       const string& filename,
                       const PixelRGB<byte>& bg)
: filename_(filename), format_(format), size_(asToon(bg.GetSize())), written_(false) {
	InitSurface();
	Clear(bg);
}

FileCanvas::FileCanvas(Format format,
                       const string& filename,
                       const MatF& bg)
	: filename_(filename),
		format_(format),
		size_(makeVector(bg.Cols(), bg.Rows())),
		written_(false) {
	InitSurface();
	DrawImageRescaled(bg);
}
	*/

FileCanvas::~FileCanvas() {
	if (!written_) {
		Write();
	}
	raw()->get_target()->finish();
}

void FileCanvas::Write() {
	raw()->get_target()->flush();
	if (format_ == PNG) {
		raw()->get_target()->write_to_png(filename_);
	} else {
		raw()->show_page();
	}
	written_ = true;
}

// static
FileCanvas::Format FileCanvas::GetFormat(const string& filename) {
	string ext = fs::extension(filename);
	for (int i = 0; i < ext.size(); i++) {
		ext[i] = tolower(ext[i]);
	}

	if (ext == ".png") {
		return PNG;
	} else if (ext == ".pdf") {
		return PDF;
	} else if (ext == ".svg") {
		return SVG;
	} else if (ext == ".ps") {
		return PS;
	} else if (ext == ".eps") {
		return EPS;
	} else {
		CHECK(false) << "Unrecognised extension: " << ext;
	}
}

void FileCanvas::InitSurface() {
	switch (format_) {
	case PNG:
		// note that FORMAT_ARGB32 below refers to pixel format,
		// which is unrelated to this->format_
		Attach(ImageSurface::create(FORMAT_ARGB32, size_[0], size_[1]));
		break;
	case PDF:
		Attach(PdfSurface::create(filename_, size_[0], size_[1]));
		break;
	case SVG:
		Attach(SvgSurface::create(filename_, size_[0], size_[1]));
		break;
	case PS:
		Attach(PsSurface::create(filename_, size_[0], size_[1]));
		break;
	case EPS:
		RefPtr<PsSurface> surf = PsSurface::create(filename_, size_[0], size_[1]);
		surf->set_eps(true);
		Attach(surf);
		break;
	}
}
}
