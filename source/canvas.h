#pragma once

#include <string>

#include <cairomm/cairomm.h>
#include <cairomm/context.h>
#include <cairomm/surface.h>

#include "common_types.h"
#include "line_segment.h"
#include "polygon-fwd.h"
#include "vw_image-fwd.h"

namespace indoor_context {

// Represents a canvas. Wraps Cairo::Context to provide vector-based methods
class Canvas {
public:
	// Constants
	static const double kDefaultDashLength;

	// Get the underlying cairo context
	inline Cairo::RefPtr<Cairo::Context> raw() { return c_; }

	// Push/pop the internal state (current position, path, transform, colour, etc)
	void PushState();
	void PopState();

	// Manipulate the path
	void MoveTo(const Vec2& p);
	void MoveTo(const Vec3& p);  // homogeneous
	void LineTo(const Vec2& p);
	void LineTo(const Vec3& p);  // homogeneous
	void Stroke();  // stroke the current path (i.e. preceeding calls to LineTo)

	// Modify the transformation matrix
	void Translate(const Vec2& t);
	void Scale(const Vec2& s);
	void Scale(double s);
	void Transform(const Mat2& m, const Vec2& t);   // pre-multiply current transform by m
	void SetTransform(const Mat2& m, const Vec2& t);   // set current transform to m

	// Set the drawing colour
	void SetColor(const PixelRGB<byte>& p);

	// Set line drawing parameters
	void SetLineWidth(double v);
	void SetLineJoin(Cairo::LineJoin v);
	void SetLineCap(Cairo::LineCap v);
	void SetLineDash(vector<double>& pattern); // alternating on/off lengths
	void SetLineDash(double length);  // alternating dashes of specified length
	void SetLineDash(bool enabled);  // enable/disable dashing (default langth is assumed)

	// Draw a dot with radius=1.0
	void DrawDot(const Vec2& c);
	void DrawDot(const Vec2& c, const PixelRGB<byte>& color);
	// Draw a dot with a specified radius
	void DrawDot(const Vec2& c, double radius);
	void DrawDot(const Vec2& c, double radius, const PixelRGB<byte>& color);

	// Draw lines
	void StrokeLine(const Vec2& a, const Vec2& b);
	void StrokeLine(const Vec2& a, const Vec2& b, const PixelRGB<byte>& color);
	void StrokeLine(const LineSeg& seg);
	void StrokeLine(const LineSeg& seg, const PixelRGB<byte>& color);

	// Draw filled polygons
	template <typename Range>
	void FillPolygon(const Range& verts);
	template <typename Range>
	void FillPolygon(const Range& verts, const PixelRGB<byte>& color);
	template <unsigned N>
	void FillPolygon(const Polygon<N>& poly);
	template <unsigned N>
	void FillPolygon(const Polygon<N>& poly, const PixelRGB<byte>& color);

	// Draw polygon outlines
	template <typename Range>
	void StrokePolygon(const Range& verts);
	template <typename Range>
	void StrokePolygon(const Range& verts, const PixelRGB<byte>& color);
	template <unsigned N>
	void StrokePolygon(const Polygon<N>& poly);
	template <unsigned N>
	void StrokePolygon(const Polygon<N>& poly, const PixelRGB<byte>& color);

	// Add a path to the context for a polygon (internal only)
	template <typename Range>
	void AddPath(const Range& verts);

	// Draw a polygon that fills the entire image with the specified color
	void Clear(const PixelRGB<byte>& bgcolor);

	// Insert an image
	void DrawImage(const ImageRGB<byte>& image, double alpha=1.0);
	// Insert an image with scaling and translation
	void DrawImage(const ImageRGB<byte>& image,
	               double scale,
	               const Vec2& translation,
	               double alpha=1.0);
	// Insert an image with an affine warp
	void DrawImage(const ImageRGB<byte>& image,
	               const Mat2& m,
	               const Vec2& translation,
	               double alpha=1.0);
	// Draw a greyscale image, rescaled so that the mesaximum value is white
	void DrawImageRescaled(const MatF& image, double alpha=1.0);
protected:
	// Protected so that this class cannot be instantiated directly
	Canvas() { };
	// Attach to the given surface
	template <typename T> void Attach(T surface);
private:
	// The underlying context
	Cairo::RefPtr<Cairo::Context> c_;
};




// A canvas that caches input and then writes to a file
class FileCanvas : public Canvas {
public:
	// Available file formats
	enum Format { PNG, PDF, SVG, PS, EPS };
	// Construct an empty canvas that will write to the specified
	// file. Format is determined by inspecting the file extension,
	// which must be one of .png, .pdf, .svg, .ps, or .eps.
	FileCanvas(const string& filename, const Vec2I& size);
	FileCanvas(const string& filename, const Vec2I& size, const PixelRGB<byte>& bgcolor);
	FileCanvas(const string& filename, const ImageRGB<byte>& bgimage);
	FileCanvas(const string& filename, const MatF& bg_greyscale);
	// Construct an empty canvas that will write to the specified file.
	// Destructor
	~FileCanvas();
	// Write the canvas to the file specified at construction
	void Write();
	// Deduce a format from a file extension
	static Format GetFormat(const string& filename);
	// Get size
	int nx() const { return size_[0]; }
	int ny() const { return size_[1]; }
	Vec2I size() const { return size_; }
private:
	// The filename passed to the constructor
	string filename_;
	// The file format
	Format format_;
	// The size
	Vec2I size_;
	// The write flag, false at construction
	bool written_;
	// Create the surface and attach to it
	void InitSurface();
};
}
