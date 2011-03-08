#pragma once

#include <iostream>

#include <VW/Image/imagergb.h>
#include <VW/Image/imagemono.h>
#include <VW/Image/imagecopy.tpp>

#include "common_types.h"

namespace indoor_context {
	using VW::ImageRef;
	using VW::ImageBase;
	using VW::ImageRGB;
	using VW::ImageMono;
	using VW::PixelRGB;
	using VW::PixelMono;

	// Classes
	/*class ImageRef;
	template <typename T> class PixelMono;
	template <typename T> class PixelRGB;
	template <typename T>	class ImageBase;
	template <typename T>	class ImageMono;
	template <typename T>	class ImageRGB;*/

	// Convenient typedefs
	typedef ImageMono<float> ImageF;
	typedef PixelMono<float> PixelF;

	// Stream outputs
	//ostream& operator<<(ostream& o, const ImageRef& p);

	// Image conversions
	void ImageConvert(const ImageRGB<byte>& in, ImageMono<float>& out);
	void ImageConvert(const ImageMono<float>& in, ImageRGB<byte>& out);

	// Clones images
	/*void ImageCopy(const ImageRGB<byte>& in, ImageRGB<byte>& out);
	void ImageCopy(const ImageRGB<byte>& in, ImageMono<float>& out);
	void ImageCopy(const ImageMono<float>& in, ImageMono<float>& out);*/

  // ugly hack for CHECK_SAME_SIZE
	inline int matrix_width(const ImageRef& p) {
		return p.x;
	}
	inline int matrix_height(const ImageRef& p) {
		return p.y;
	}
}
