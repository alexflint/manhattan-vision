#include <GL/gl.h>
#include <GL/glut.h>

#include <boost/filesystem.hpp>

#include <VW/Image/imagecopy.tpp>

#include "image_bundle.h"
#include "common_types_vw.h"
#include "gl_utils.tpp"
#include "glut_window.h"

namespace indoor_context {
	ImageBundle::ImageBundle() : hsvdirty(true), monodirty(true), texdirty(true) {
	}

	ImageBundle::ImageBundle(const string& filename)
		: hsvdirty(true), monodirty(true) {
		Load(filename);
	}


	ImageBundle::ImageBundle(const char* filename)
		: hsvdirty(true), monodirty(true) {
		Load(filename);
	}

	/*ImageBundle::ImageBundle(const fs::path& filename)
		: hsvdirty(true), monodirty(true) {
		Load(filename);
	}*/

	bool ImageBundle::contains(const ImageRef& ir) const {
		return ir.x >= 0 && ir.y >= 0 && ir.x < nx() && ir.y < ny();
	}

	void ImageBundle::Load(const string& filename) {
		ReadImage(filename, rgb);
		Invalidate();
	}

	void ImageBundle::Load(const char* filename) {
		ReadImage(filename, rgb);
		Invalidate();
	}

	/*void ImageBundle::Load(const fs::path& filename) {
		ReadImage(filename.string(), rgb);
		Invalidate();
	}*/

	void ImageBundle::Unload() {
		if (rgb.IsAlloced()) rgb.FreeImageData();
		if (hsv.IsAlloced()) hsv.FreeImageData();
		if (mono.IsAlloced()) mono.FreeImageData();
	}

	void ImageBundle::BuildMono() const {
		if (monodirty) {
			ImageCopy(rgb, mono);
			monodirty = false;
		}
	}

	void ImageBundle::BuildHSV() const {
		if (hsvdirty) {
			DLOG << "Warning: using ImageCopy for RGB -> HSV, this doesn't work properly!";
			ImageCopy(rgb, hsv);
			hsvdirty = false;
		}
	}

	// Load this image as a texture into GL
	unsigned ImageBundle::LoadGLTexture(unsigned texId) const {
		// VW images use alpha=0 to indicate full opacity, whereas in
		// OpenGL the default is that alpha=0 indicates full
		// transparency. If you have GL_BLEND enabled then you must either do
		// 	 glBlendFunc(GL_ONE_MINUS SRC_ALPHA, GL_SRC_ALPHA);
		// or
		//   glDisable(GL_BLEND)
		// when rendering this texture
		if (texId == -1) {
			glGenTextures(1, &gl_texture_id);
		} else {
			gl_texture_id = texId;
		}
		CHECK(GlutWindow::InGlutThread());
		glBindTexture(GL_TEXTURE_2D, gl_texture_id);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, 4, nx(), ny(), 
								 0, GL_BGRA, GL_UNSIGNED_BYTE,
								 rgb.GetRawBuffer());
		glError();
		return gl_texture_id;
	}

	void ImageBundle::BindGLTexture() const {
		if (texdirty) {
			texdirty = false;
			gl_texture_id = LoadGLTexture();
		}
		glBindTexture(GL_TEXTURE_2D, gl_texture_id);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glError();
	}

	void ImageBundle::Invalidate() {
		hsvdirty = true;
		monodirty = true;
		texdirty = true;
	}
}
