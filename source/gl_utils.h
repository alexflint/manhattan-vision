#include "common_types.h"
#include "camera.h"

#include <GL/gl.h>

#define glError() indoor_context::glErrorInternal(__FILE__, __LINE__);

namespace indoor_context {
	// Report GL errors
	void glErrorInternal(const char* file, int line);

	// Get current GL modelview matrix
	Mat4 GetGLModelView();
	// Get current GL projection matrix
	Mat4 GetGLProjection();
	// Get current viewport dimensions
	Vec4 GetGLViewport();

	// Configure both the modelview and projection matrices for the
	// given camera's world coordinate system.
	void ConfigureGLForCamera(const PosedCamera& camera);
	// Configure the current projection matrix to match the given camera
	// Note: changes the matrix mode
	void ConfigureGLProjectionForCamera(const CameraBase& camera);
	// Configure the current modelview matrix for a camera's eye
	// coordinate system (this is almost an identity transform). Note:
	// changes the matrix mode
	void ConfigureGLModelViewForEyeCoords();
	// Configure the current modelview matrix for a camera's world
	// coordinate system (this is almost an identity transform). Note:
	// changes the matrix mode
	void ConfigureGLModelViewForWorldCoords(const PosedCamera& camera);

	// Renders a textured quad on the z=1 plane with the specified dimensions
	void RenderTexturedQuad(const Bounds2D<>& quad,
													GLuint texture_id);

	// Renders the given image so that it fills the viewport. Leaves the
	// modelview and projection matrices unchanged.
	void RenderFullScreen(const ImageBundle& image);
	// Renders the given image so that it fills the viewport. Leaves the
	// modelview and projection matrices unchanged.
	void RenderFullScreen(GLuint texture_id);

	// Transform the current GL matrix
	void TransformGL(const toon::SE3<>& T);
}
