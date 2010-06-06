#include "gl_utils.tpp"

#include <GL/glut.h>

namespace indoor_context {
	using namespace toon;

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
