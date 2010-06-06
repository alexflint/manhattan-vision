#pragma once

#include "common_types.h"

namespace indoor_context {

// Represents an image transformed into various color spaces. The
// image is initially read into the RGB version, and the mono and HSV
// versions are generated with BuildMono() and BuildHSV(), which only
// do work if those transformations have not already been
// generated. To allow these lazy conversions the HSV and Mono images
// are marked mutable.
class ImageBundle {
public:
	ImageRGB<byte> rgb;
	// The const semantics of this class preserve the constness of the
	// image itself. The following are thought of as alternate
	// representations of the same image, hence they are marked mutable
	mutable ImageHSV<byte> hsv;
	mutable bool hsvdirty;  // RGB has been modified since HSV was built
	mutable ImageF mono;
	mutable bool monodirty;  // RGB has been modified since M was built
	mutable unsigned gl_texture_id;  // texture name if this image has been loaded
	mutable bool texdirty;
	// Initialize empty
	ImageBundle();
	// Initialize from an image file
	ImageBundle(const string& filename);
	// Initialize from an image file
	ImageBundle(const char* filename);
	// Initialize from an image file -- commented for compile speed
	//ImageBundle(const fs::path& filename);
	// Get image width
	inline int nx() const { return rgb.GetWidth(); }
	// Get image height
	inline int ny() const { return rgb.GetHeight(); }
	// Get image size
	inline ImageRef sz() const { return rgb.GetSize(); }
	// True iff the location is within the image bounds
	bool contains(const ImageRef& ir) const;
	// Read an image file into this->rgb
	void Load(const string& filename);
	// Read an image file into this->rgb
	void Load(const char* filename);
	// Unload all images
	void Unload();
	// Read an image file into this->rgb  -- commented for compile speed
	//void Load(const fs::path& filename);
	// Convert this->rgb to mono and store in this->m
	void BuildMono() const;
	// Convert this->rgb to HSV and store in this->hsv
	void BuildHSV() const;
	// Load the texture into memory under the specified texture ID (or
	// generate a new one if id=-1). For multi-window applications
	// the texture can only be used in the window it was loaded
	// for. Consider using Viewer3D::LoadTexture.
	unsigned LoadGLTexture(unsigned texId=-1) const;
	// Bind this image as the current texture. This may involve loading
	// the texture into memory if it has not already been loaded. Only
	// use this in single window applications.
	void BindGLTexture() const;
	// Set the dirty flag so that future invokations of Build* will
	// re-build the respective image
	void Invalidate();
private:
	ImageBundle(const ImageBundle& other) {}  // disabled
};

}
