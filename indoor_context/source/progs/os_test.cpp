#include "glut_window.h"
#include "common_types.h"

#include "gl_utils.tpp"
#include "image_utils.tpp"

using namespace indoor_context;

class OffScreenRenderer

bool rendered;
GlutWindow win;

void render() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, 100, 0, 100, -100, 100);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glColorP(Colors::red());
	GL_PRIMITIVE(GL_QUADS) {
		glVertex3f(10,20,5);
		glVertex3f(10,40,5);
		glVertex3f(30,40,5);
		glVertex3f(30,20,5);
	}

	glColorP(Colors::blue());
	GL_PRIMITIVE(GL_QUADS) {
		glVertex3f(20,35,10);
		glVertex3f(20,60,10);
		glVertex3f(80,60,10);
		glVertex3f(80,35,10);
	}

	win.OutputFrameBuffer("out/grab1.png");
	rendered = true;
}

int main(int argc, char **argv) {
	rendered = false;
	win.SetDisplayMode(GLUT_SINGLE | GLUT_DEPTH);
	win.Display.add(&render);

  // Hide the window as soon as it is created. Seems there is no
  // better way to do this.
	win.VisibilityChanged.add(bind(&GlutWindow::Hide, &win));

	win.Run();


	return 0;
}
