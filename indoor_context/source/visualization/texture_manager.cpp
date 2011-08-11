#include <GL/gl.h>

#include "texture_manager.h"
#include "glut_window.h"
#include "gl_utils.h"

namespace indoor_context {
	void TextureManager::Select(const ImageBundle* image) {
		glBindTexture(GL_TEXTURE_2D, LoadOrLookup(image));
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glError();
	}

	GLuint TextureManager::LoadOrLookup(const ImageBundle* image) {
		if (textureIds_.find(image) == textureIds_.end()) {
			textureIds_[image] = image->LoadGLTexture();
		}
		return textureIds_[image];
	}

	GLuint TextureManager::Reload(const ImageBundle* image) {
		GLuint texId = -1;
		if (textureIds_.find(image) != textureIds_.end()) {
			texId = textureIds_[image];
		}
		textureIds_[image] = image->LoadGLTexture(texId);
	}

	void TextureManager::Delete(const ImageBundle* image) {
		map<const ImageBundle*,unsigned>::iterator it = textureIds_.find(image);
		if (it != textureIds_.end()) {
			GLuint texId = textureIds_[image];
			glDeleteTextures(1, &texId);
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
