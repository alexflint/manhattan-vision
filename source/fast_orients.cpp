#include <cmath>

#include <iostream>

#include <LU.h>

#include "vars.h"
#include "common_types.h"
#include "timer.h"
#include "image_bundle.h"
#include "map.h"
#include "glut_window.h"
#include "viewer3d.h"

#include "gl_utils.tpp"

#include "image_utils.tpp"
#include "math_utils.tpp"

using namespace GVars3;
using namespace indoor_context;
using namespace toon;

class FastOrientsGUI {
private:
	const Map& map;
	int index;
	ImageBundle frame;
	GlutWindow window;
	TextureManager texman;
	Matrix<3> H[3];
	Matrix<3> Hinv[3];
public:
	FastOrientsGUI(const Map& m)
		: map(m),
			index(0) {
		window.SetSize(ImageRef(640,480));
		window.Display.add(bind(&FastOrientsGUI::OnDraw, this));
		window.Idle.add(bind(&FastOrientsGUI::OnIdle, this));
	}

	void MultMatrix(const Matrix<3>& m) {
		double p[16];
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				p[i*4+j] = m[i][j];
			}
			p[i*4+3] = 0;
		}
		p[15] = 1;
		glMultMatrixd(p);
	}

	void OnDraw() {
		//glMatrixMode(GL_PROJECTION);
		if (frame.rgb.IsAlloced()) {
			glMatrixMode(GL_PROJECTION);
			GL_MATRIX_SCOPE {
				glLoadIdentity();
				glRotatef(180,0,0,1);
				glScalef(0.5, 0.5, 1.0);
				glTranslatef(1, 1, 0.0);
				texman.Render(&frame, &window);

				for (int i = 0; i < 3; i++) {
					glLoadIdentity();
					//DREPORT(i,Hinv[i],Hinv[i]*makeVector(1,1,1));
					//DREPORT(Hinv[i]*makeVector(-1,-1,1));
					MultMatrix(H[i]);
					glRotatef(180,0,0,1);
					glScalef(0.5, 0.5, 1.0);
					glTranslatef(-1, -1, 0.0);
					texman.Render(&frame, &window);
				}
			}
		}
	}

	void ComputeWarps(const SE3<>& pose) {
		for (int i = 0; i < 3; i++) {
			Vector<3> up = col(pose.get_rotation(), i);

			Matrix<3> R_up;
			R_up[0] = up ^ GetAxis<3>(2);
			R_up[1] = up;
			R_up[2] = R_up[0] ^ R_up[1];

			// T_centre projects the centre of the original image projects to
			// the centre of the new image
			Matrix<3> T_centre = Identity;
			T_centre.slice<0,2,2,1>() = -project(R_up*makeVector(0,0,1)).as_col();

			// Compute the warping homographies to transform the image so that
			// vertical in the world is vertical in the image.
			H[i] = T_centre * R_up;
			Hinv[i] = LU<>(H[i]).get_inverse();
		}
	}

	void OnIdle() {
		if (index < map.frame_specs.size()) {
			const Frame& fs = map.frame_specs[index];
			frame.Load(fs.filename);
			texman.Delete(&frame);
			ComputeWarps(fs.CfW);
		}
		index++;
		window.Invalidate();
	}

	void Run() {
		window.Run();
	}
};




int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return -1;
	}

	Map map;
	map.Load();

	FastOrientsGUI gui(map);
	gui.Run();

	return 0;
}
