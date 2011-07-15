#include "vw_image_io.h"

#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/gil/extension/io/jpeg_dynamic_io.hpp>
#include <boost/gil/image_view.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include "common_types.h"
#include "vw_image.tpp"

// For GIL's typedef conventions see
// http://www.boost.org/doc/libs/1_38_0/libs/gil/doc/html/giltutorial.html#AppendixConventionSec

namespace indoor_context {
	using namespace boost::gil;

	// Invert alpha channel in an RGB image
	void InvertAlpha(ImageRGB<byte>& image) {
		for (int y = 0; y < image.GetHeight(); y++) {
			PixelRGB<byte>* row = image[y];
			for (int x = 0; x < image.GetWidth(); x++) {
				row[x].alpha = 255-row[x].alpha;
			}
		}
	}

	// Minimal mock of VW imageio routines
	void ReadImage(const std::string& file, ImageRGB<byte>& image) {
		CHECK_PRED1(fs::exists, file);

		// TODO: find a more reliable way to determine file format
#if BOOST_VERSION >= 104400
		string ext = fs::path(file).extension().string();
#else
		string ext = fs::path(file).extension();
#endif
		bool jpeg = (ext == ".jpg" || ext == ".JPG" || ext == ".jpeg" || ext == ".JPEG");

		// Get size
		point2<ptrdiff_t> size;
		if (jpeg) {
			size = jpeg_read_dimensions(file);
		} else {
			size = png_read_dimensions(file);
		}

		// Allocate image data
		image.AllocImageData(size.x, size.y);
		rgba8_view_t v = interleaved_view(image.GetWidth(),
																			image.GetHeight(),
																			(rgba8_pixel_t*)image.GetImageBuffer(),
																			image.GetWidth()*sizeof(PixelRGB<byte>));

		// Load the image
		if (jpeg) {
			jpeg_read_and_convert_view(file, v);
		} else {
			png_read_and_convert_view(file, v);
		}

		// GIL uses 255=opaque but we use 0=opaque
		InvertAlpha(image);
	}

	void WriteImage(const std::string& file, const ImageRGB<byte>& image) {
		rgba8c_view_t v = interleaved_view(image.GetWidth(),
																			 image.GetHeight(),
																			 (const rgba8_pixel_t*)image.GetImageBuffer(),
																			 image.GetWidth()*sizeof(PixelRGB<byte>));
		// Here we use a hack to work around the fact that GIL uses 255=opaque but we use 0=opaque
		InvertAlpha(const_cast<ImageRGB<byte>&>(image));
		png_write_view(file, v);
		InvertAlpha(const_cast<ImageRGB<byte>&>(image));
	}

 	void WriteImage(const std::string& file, const ImageMono<float>& image) {
		// Temporary hack until we find out how to do this properly with gil
		ImageRGB<byte> canvas;
		ImageConvert(image, canvas);
		WriteImage(file, canvas);
	}
}
