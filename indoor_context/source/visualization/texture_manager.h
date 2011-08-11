#pragma once

#include <map>

#include "common_types.h"
#include "image_bundle.h"
#include "glut_window.h"

namespace indoor_context {
	// Manages loading, unloading, and binding GL textures
	class TextureManager {
	public:
		// Select the texture for a given image. If the image has not yet
		// been loaded into GL then it will be loaded under a generated
		// texture ID.
		void Select(const ImageBundle* image);
		// Load an image as a GL texture or look up its texture ID if it
		// has already been loaded.
		GLuint LoadOrLookup(const ImageBundle* image);
		// Reload a texture into GL (or load it for the first time if has
		// not been loaded).
		GLuint Reload(const ImageBundle* image);
		// Remove a texture from GL (or do nothing if it has not been loaded)
		void Delete(const ImageBundle* image);
		// Draw a texture
		//void Render(const ImageBundle* image, const GlutWindow* window);
		// Draw a texture full-screen
		//void RenderFullScreen(const ImageBundle* image, const GlutWindow* window);
	private:
		// Map of images to texture IDs
		map<const ImageBundle*,unsigned> textureIds_;
	};
}
