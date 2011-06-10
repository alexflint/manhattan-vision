#include "glut_window.h"

using namespace std;
using namespace indoor_context;

void keyboard(unsigned char c, int x, int y) {
	cout << "Recieved regular key '" << c << "' (" << static_cast<int>(c) << ")" << endl;
}

void special(int key, int x, int y) {
	cout << "Recieved special key " << key << endl;
}

int main(int argc, char **argv) {
	GlutWindow win;
	win.Create();
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(special);
	win.Run();

	return 0;
}
