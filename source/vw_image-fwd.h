#pragma once

#include <iostream>

#include "common_types.h"

namespace indoor_context {
	// Classes
	class ImageRef;
	template <typename T> class PixelMono;
	template <typename T> class PixelRGB;
	template <typename T>	class ImageBase;
	template <typename T>	class ImageMono;
	template <typename T>	class ImageRGB;

	// Stream outputs
	ostream& operator<<(ostream& o, const ImageRef& p);

	// Image copying
	/*void ImageCopy(const ImageRGB<byte>& in, ImageRGB<byte>& out);
	void ImageCopy(const ImageRGB<byte>& in, ImageMono<float>& out);
	void ImageCopy(const ImageMono<float>& in, ImageMono<float>& out);*/

	// Image conversions
	void ImageConvert(const ImageRGB<byte>& in, ImageMono<float>& out);
	void ImageConvert(const ImageMono<float>& in, ImageRGB<byte>& out);

	// Convenient typedefs
	typedef ImageMono<float> ImageF;
	typedef PixelMono<float> PixelF;
}
