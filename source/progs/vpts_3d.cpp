#include <cmath>

#include <iostream>

#include <VW/Events/events.h>
#include <VWGL/Display/displayglut.h>
#include <VWGL/Display/glutloop.h>
#include <VWGL/Display/drawgl.h>
#include <VWGL/Display/threedtoolglut.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "misc.h"
#include "common_types_vw.h"
#include "vanishing_points.h"
#include "timer.h"
#include "image_bundle.h"
#include "image_utils.tpp"

using namespace VWEvents;

class VanPointGUI : public EventHandler {
private:
	const ptr_vector<ImageBundle>& images;
	ptr_vector<VanishingPointsEM> vpts;
	ImageRGB<byte> frame;
	ThreeDToolGLUT display;
	int nx, ny;
public:
	VanPointGUI(const ptr_vector<ImageBundle>& ims)
		: images(ims),
			display(100, 100) {
		nx = ny = 0;
		for (int i = 0; i < images.size(); i++) {
			nx += images[i].nx()+100;
			ny = max(ny, images[i].ny()+100);
		}
		vpts.resize(images.size());

		frame.AllocImageData(nx, ny);

		display.DrawEvent.Attach(this, &VanPointGUI::OnDraw);
		display.KeyboardEvent.Attach(this, &VanPointGUI::OnKeyboard);
		//display.ClickEvent.Attach(this, &VanPointGUI::OnClick);
		Recompute();
	}

	void Recompute() {
		DLOG << "\n\nRecomputing\n\n";
		ImageRGB<byte> canvas;
		int x_offs = 0;
		for (int i = 0; i < images.size(); i++) {
			vpts[i].Compute(images[i]);
			ResizeImage(canvas, images[i].nx()+100, images[i].ny()+100);
			vpts[i].DrawVanPointViz(canvas);
			CopyImageInto(canvas, 0, x_offs, frame);
			x_offs += canvas.GetWidth();
		}
		display.RequestDraw();
	}

	void OnDraw(bool b) {
		if (display.CameraWidth() != nx || display.CameraHeight() != ny) {
			display.SetCameraSize(nx, ny);
		}
		if (frame.IsAlloced()) {
			display.DrawCinemaScreen(1.0, frame);
			//display.Flush();
		}
	}

	void OnClick(int x, int y, int button) {
		for (int i = 0; i < images.size(); i++) {
			if (x < images[i].nx()+100) {
				int imx = x-50;
				int imy = y-50;
				for (int j = 0; j < vpts[i].lines.segments.size(); j++) {
					const LineSegment& seg = *vpts[i].lines.segments[j];
					BOOST_FOREACH(const ImageRef& p, seg.pixels) {
						int dx = p.x - imx;
						int dy = p.y - imy;
						if (abs(dx) + abs(dy) <= 2) {
							DLOG << "Image " << i << ", segment " << j;
							break;
						}
					}
				}
				break;
			} else {
				x -= images[i].nx()+100;
			}
		}
		DLOG << x << " " << y;
	}

	void OnKeyboard(int x, int y, int key) {
		DREPORT(key);
		DREPORT(x);
		DREPORT(y);
		switch (key) {
		case 'w':
			// Write out the current frame
			WriteImage("frame.png", frame);
			cout << "Wrote frame.png\n";
			break;

		case 'r':
			ReloadVars();
			Recompute();
			break;

		case 'q':
		case 27: 
			exit(0);
			break;
		}
	}

	void Run() {
		GLUTLoop();
	}
};




int main(int argc, char **argv) {
  GLUTInit(argc, argv);
	InitVars(argc, argv);
	if (argc < 2) {
		DLOG << "Usage: "<<argv[0]<<" IMAGE1 IMAGE2 ...";
		return -1;
	}

	ptr_vector<ImageBundle> images;
	for (int i = 1; i < argc; i++) {
		images.push_back(new ImageBundle(argv[i]));
	}
	VanPointGUI gui(images);
	gui.Run();

	return 0;
}
