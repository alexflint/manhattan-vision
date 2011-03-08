#include "common_types.h"

#include "vw_image-fwd.h"

namespace indoor_context {
	// Read image from a file
	void ReadImage(const std::string& file, ImageRGB<byte>& image);

	// Read an RGB image from a file
	// Warning: this method modifies the image data internally, then
	// changes it back before returning. This will only be an issue in
	// multi-threaded applications.
	void WriteImage(const std::string& file, const ImageRGB<byte>& image);

	// Read an mono image from a file
	void WriteImage(const std::string& file, const ImageMono<float>& image);
}
