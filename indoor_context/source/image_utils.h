#pragma once

#include "common_types.h"
#include "colors.h"

#include "line_segment.h"
#include "vw_image-fwd.h"

namespace indoor_context {

	// Vizualize a segmentation by assigning a colour to each region
	void DrawSegmentation(const MatI& mat,
												ImageRGB<byte>& output);
	void DrawSegmentation(const MatI& mat,
												const vector<PixelRGB<byte> >& colors,
												ImageRGB<byte>& output);

	// Draw a spot with specified side length and color at X in IMAGE.
	void DrawSpot(ImageRGB<byte>& image,
								const Vec2& x,
								const PixelRGB<byte>& color,
								const int size);

	// Draw an alpha-blended line
	void DrawLine(ImageRGB<byte>& canvas,
								const Vec2& start,
								const Vec2& end,
								const PixelRGB<byte>& color,
								const float alpha=1.0);

	// Draw an alpha-blended line
	void DrawLine(ImageRGB<byte>& canvas,
								const LineSeg& seg,
								const PixelRGB<byte>& color);

	// Draw a line, clipped to the image boundary
	void DrawLineClipped(ImageRGB<byte>& canvas,
											 const Vec2& start,
											 const Vec2& end,
											 const PixelRGB<byte>& color,
											 double alpha=1.0);

	// Draw a line, clipped to the image boundary
	void DrawLineClipped(ImageRGB<byte>& canvas,
											 const LineSeg& seg,
											 const PixelRGB<byte>& color,
											 double alpha=1.0);

	// For each pixel>0 in stencil, set the corresponding pixel in canvas
	// to the specified color.
	void DrawStencil(const MatI& stencil,
									 ImageRGB<byte>& canvas,
									 const PixelRGB<byte>& color);


	// Draw orientations in red (vertical 1), green (vertical 2), and blue (horizontal)
	void DrawOrientations(const MatI& orients,
												ImageRGB<byte>& canvas,
												double alpha=1.0);

	// Output an orientations image
	void WriteOrientationImage(const string& filename,
														 const MatI& orients);
	void WriteOrientationImage(const string& filename,
														 const MatI& orients,
														 const ImageRef& sz);

	// Blend two colors in propotions A to 1-A
	PixelRGB<byte> Blend(const PixelRGB<byte>& p1,
											 const PixelRGB<byte>& p2,
											 const float alpha);

	// Blend two colors in propotions A to 1-A
	void BlendWith(PixelRGB<byte>& p1,
								 const PixelRGB<byte>& p2,
								 const float alpha=0.5);

	// Set the alpha of every pixel to zero
	void ResetAlpha(ImageRGB<byte>& image);

	// Draw an image representation of a histogram. The image will be
	// resized to N x aspect*N pixels, where N=hist.size().
	void DrawHistogram(ImageRGB<byte>& canvas,
										 const vector<int>& hist,
										 float aspect);

	// Draw red pixels wherever M is negative
	void DrawNegatives(const MatD& m, ImageRGB<byte>& canvas);
}
