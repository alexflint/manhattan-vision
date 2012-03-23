#include <boost/filesystem.hpp>

#include "image_bundle.h"
#include "common_types.h"
#include "vw_image_io.h"

namespace indoor_context {
	ImageBundle::ImageBundle() : monodirty(true) {
	}

	ImageBundle::ImageBundle(const string& filename) : monodirty(true) {
		Load(filename);
	}

	bool ImageBundle::contains(const ImageRef& ir) const {
		return ir.x >= 0 && ir.y >= 0 && ir.x < nx() && ir.y < ny();
	}

	void ImageBundle::Load(const string& filename) {
		ReadImage(filename, rgb);
		Invalidate();
	}

	void ImageBundle::Unload() {
		if (rgb.IsAlloced()) rgb.FreeImageData();
		if (mono.IsAlloced()) mono.FreeImageData();
	}

	bool ImageBundle::loaded() const {
		return rgb.IsAlloced();
	}

	void ImageBundle::BuildMono() const {
		if (monodirty) {
			CHECK(loaded());
			ImageConvert(rgb, mono);
			monodirty = false;
		}
	}

	// Save the current image in this->mono
	void ImageBundle::SaveMono() {
		monodirty = false;
	}

	void ImageBundle::Invalidate() {
		monodirty = true;
	}
}
