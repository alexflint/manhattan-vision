#include "gl_utils.h"
#include "gl_utils.tpp"

#include <GL/glut.h>

namespace indoor_context {
	using namespace toon;

	void ConfigureGLForCamera(const PosedCamera& camera) {
		ConfigureGLProjectionForCamera(camera.camera());
		ConfigureGLModelViewForWorldCoords(camera);
	}

	void ConfigureGLProjectionForCamera(const CameraBase& camera) {
		double znear = 1e-2, zfar = 1e+2;
		Vec2 tl = camera.retina_bounds().tl();
		Vec2 br = camera.retina_bounds().br();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glFrustum(tl[0]*znear, br[0]*znear, tl[1]*znear, br[1]*znear, znear, zfar);
	}

	void ConfigureGLModelViewForEyeCoords() {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glScalef(1,-1,-1);  // look down the positive Z axis, and put origin at top left
	}

	void ConfigureGLModelViewForWorldCoords(const PosedCamera& camera) {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glScalef(1,-1,-1);  // look down the positive Z axis, and put origin at top left
		TransformGL(camera.pose());
	}

	void RenderTexturedQuad(const Bounds2D<>& quad,
													GLuint texture_id) {
		glColorP(Colors::white());
		glBindTexture(GL_TEXTURE_2D, texture_id);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		WITHOUT(GL_BLEND) WITH(GL_TEXTURE_2D) GL_PRIMITIVE(GL_QUADS) {
			glTexCoord2f(0, 0);
			glVertex3f(quad.tl()[0], quad.tl()[1], 1);
			glTexCoord2f(0, 1);
			glVertex3f(quad.tl()[0], quad.br()[1], 1);
			glTexCoord2f(1, 1);
			glVertex3f(quad.br()[0], quad.br()[1], 1);
			glTexCoord2f(1, 0);
			glVertex3f(quad.br()[0], quad.tl()[1], 1);
		}
		glError();
	}

	void RenderFullScreen(const ImageBundle& image) {
		RenderFullScreen(image.LoadGLTexture());
	}

	void RenderFullScreen(GLuint texture_id) {
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();

		glMatrixMode(GL_MODELVIEW);
		GL_MATRIX_SCOPE {
			ConfigureGLModelViewForEyeCoords();
			RenderTexturedQuad(Bounds2D<>(-1,1,-1,1), texture_id);
		}

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	}

	void TransformGL(const SE3<>& T) {
		Mat4 M = Identity;
		M.slice<0,0,3,3>() = T.get_rotation().get_matrix();
		M.slice<0,3,3,1>() = T.get_translation().as_col();
		TransformGL(M);
	}

	void glErrorInternal(const char* file, int line) {
		GLenum err = glGetError();
		while (err != GL_NO_ERROR) {
			fprintf(stderr, "glError: %s caught at %s:%u\n",
							(char*)gluErrorString(err), file, line);
			err = glGetError();
		}
	}

	Matrix<4,4> PackMatrix(double* p) {
		Matrix<4,4> m;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				m[i][j] = p[j*4+i];
			}
		}
		return m;
	}

	Matrix<4,4> GetGLMatrix(GLenum pname) {
		scoped_array<double> buf(new double[16]);
		glGetDoublev(pname, buf.get());
		return PackMatrix(buf.get());
	}

	Matrix<4,4> GetGLModelView() {
		return GetGLMatrix(GL_MODELVIEW_MATRIX);
	}

	Matrix<4,4> GetGLProjection() {
		return GetGLMatrix(GL_PROJECTION_MATRIX);
	}

	Vector<4> GetGLViewport() {
		Vector<4> v;
		glGetDoublev(GL_VIEWPORT, &v[0]);
		return v;
	}

	/*
	// static
	GlutHandlers* GlutHandlers::instance() {
	static scoped_ptr<GlutHandlers> p;
	if (p.get() == NULL) {
	p.reset(new GlutHandlers());
	}
	return p.get();
	}

	// static
	void GlutHandlers::install() {
		instance();  // Make sure the single instance is instantiated
		glutDisplayFunc(GlutHandlers::Handle_OnDisplay);
		glutKeyboardFunc(GlutHandlers::Handle_OnKeyboard);
		glutMouseFunc(GlutHandlers::Handle_OnMouse);
		glutMotionFunc(GlutHandlers::Handle_OnMotion);
		glutPassiveMotionFunc(GlutHandlers::Handle_OnPassiveMotion);
		glutReshapeFunc(GlutHandlers::Handle_OnReshape);
	}

	void GlutHandlers::CheckInstance() {
		CHECK(instance) << "A glut handler was invoked but there is no instance.";
	}

	// static
	void GlutHandlers::Handle_OnDisplay() {
		CheckInstance();
		instance()->Display.fire();
	}

	// static
	void GlutHandlers::Handle_OnKeyboard(unsigned char c, int x, int y) {
		CheckInstance();
		instance()->Keyboard.fire(c, x, y);
	}

	// static
	void GlutHandlers::Handle_OnMouse(int button, int state, int x, int y) {
		CheckInstance();
		instance()->Mouse.fire(button, state, x, y);
	}

	// static
	void GlutHandlers::Handle_OnMotion(int x, int y) {
		CheckInstance();
		instance()->Motion.fire(x, y);
	}

	// static
	void GlutHandlers::Handle_OnPassiveMotion(int x, int y) {
		CheckInstance();
		instance()->PassiveMotion.fire(x, y);
	}

	// static
	void GlutHandlers::Handle_OnReshape(int w, int h) {
		CheckInstance();
		instance()->Reshape.fire(w, h);
	}
	*/
}
