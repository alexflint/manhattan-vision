#include <GL/gl.h>

#include "texture_manager.h"
#include "glut_window.h"
#include "gl_utils.h"

namespace indoor_context {
	// Load an image as a GL texture
	unsigned TextureManager::LoadTexture(const ImageBundle& image) {
		// VW images use alpha=0 to indicate full opacity, whereas in
		// OpenGL the default is that alpha=0 indicates full
		// transparency. If you have GL_BLEND enabled then you must either do
		// 	 glBlendFunc(GL_ONE_MINUS SRC_ALPHA, GL_SRC_ALPHA);
		// or
		//   glDisable(GL_BLEND)
		// when rendering this texture
		CHECK(GlutWindow::InGlutThread());

		unsigned textureId;
		glGenTextures(1, &textureId);

		glBindTexture(GL_TEXTURE_2D, textureId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, 4, image.nx(), image.ny(), 
								 0, GL_RGBA, GL_UNSIGNED_BYTE,
								 image.rgb.GetImageBuffer());
		glError();
		return textureId;
	}

	void TextureManager::BindTexture(unsigned textureId) {
		glBindTexture(GL_TEXTURE_2D, textureId);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glError();
	}

	void TextureManager::Select(const ImageBundle* image) {
		BindTexture(LoadOrLookup(image));
	}

	GLuint TextureManager::LoadOrLookup(const ImageBundle* image) {
		if (textureIds_.find(image) == textureIds_.end()) {
			textureIds_[image] = LoadTexture(*image);
		}
		return textureIds_[image];
	}

	GLuint TextureManager::Reload(const ImageBundle* image) {
		GLuint textureId = -1;
		if (textureIds_.find(image) != textureIds_.end()) {
			textureId = textureIds_[image];
		}
		textureIds_[image] = LoadTexture(*image);  // should re-use old texture id!
	}

	void TextureManager::Delete(const ImageBundle* image) {
		map<const ImageBundle*,unsigned>::iterator it = textureIds_.find(image);
		if (it != textureIds_.end()) {
			GLuint textureId = textureIds_[image];
			glDeleteTextures(1, &textureId);
			textureIds_.erase(it);
		}
	}

	/*void TextureManager::Render(const ImageBundle* image,
															const GlutWindow* window) {
		// Select the window and the texture
		window->Select();
		Select(image);

		// Render the textured quad
		glShadeModel(GL_FLAT);
		WITHOUT(GL_DEPTH_TEST) WITHOUT(GL_COLOR_MATERIAL) WITH(GL_TEXTURE_2D) {
			glBegin(GL_QUADS);
			glTexCoord2f(0., 0.);
			glVertex3f(-1, -1, 1.);
			glTexCoord2f(0., 1.);
			glVertex3f(-1, 1, 1.);
			glTexCoord2f(1., 1.);
			glVertex3f(1,1, 1.);
			glTexCoord2f(1., 0.);
			glVertex3f(1, -1, 1.);
			glEnd();
		}
		glShadeModel(GL_SMOOTH);
	}

	void TextureManager::RenderFullScreen(const ImageBundle* image,
																				const GlutWindow* window) {
		GL_MATRIX_SCOPE {
			glLoadIdentity();
			glRotatef(180.0, 0.0, 0.0, 1.0);
			Render(image, window);
		}
		}*/
}
