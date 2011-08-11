#include <cmath>

#include <iostream>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <VW/Events/events.h>
#include <VWGL/Display/displayglut.h>
#include <VWGL/Display/glutloop.h>
#include <VWGL/Display/drawgl.h>

#include <gvars3/GUI.h>

#include <se3.h>

#include "misc.h"
#include "common_types_vw.h"
#include "vanishing_points.h"
#include "timer.h"
#include "image_bundle.h"
#include "unwarped_image.h"

#include "image_utils.tpp"

using namespace indoor_context;
using namespace VWEvents;


double LinearXform(const double p,
									 const double src_min,
									 const double src_max,
									 const double dest_min,
									 const double dest_max) {
	return dest_min + (dest_max-dest_min) * (p-src_min) / (src_max-src_min);
}

TooN::Vector<2> LinearXform(const TooN::Vector<2>& p,
														const TooN::Vector<2>& src_tl,
														const TooN::Vector<2>& src_br,
														const TooN::Vector<2>& dest_tl,
														const TooN::Vector<2>& dest_br) {
	return TooN::makeVector(LinearXform(p[0], src_tl[0], src_br[0], dest_tl[0], dest_br[0]),
													LinearXform(p[1], src_tl[1], src_br[1], dest_tl[1], dest_br[1]));
}


TooN::Vector<2> fromVNL(const Vec2D& v) {
	return TooN::makeVector(v[0], v[1]);
}

TooN::Vector<2> HProj(const TooN::Vector<3>& v) {
	return TooN::makeVector(v[0]/v[2], v[1]/v[2]);
}

TooN::Vector<3> HUnproj(const TooN::Vector<2>& v) {
	return TooN::makeVector(v[0], v[1], 1);
}

Vec2D fromToon(const TooN::Vector<2>& v) {
	return Vec2D(v[0], v[1]);
}


class VanPointGUI : public EventHandler {
private:
	ptr_vector<KeyFrame>& kfs;
	ImageRGB<byte> canvas;
	DisplayGLUT display;
	int nx, ny;
	TooN::Vector<3> worldvpt;
public:
	VanPointGUI(ptr_vector<KeyFrame>& keyframes)
		: kfs(keyframes),
			display(1, 1, "Vanishing Points GUI") {
		nx = ny = 0;
		for (int i = 0; i < kfs.size(); i++) {
			nx += kfs[i].image.nx()+100;
			ny = max(ny, kfs[i].image.ny()+100);
		}

		canvas.AllocImageData(nx, ny);

		display.DrawEvent.Attach(this, &VanPointGUI::OnDraw);
		display.KeyboardEvent.Attach(this, &VanPointGUI::OnKeyboard);
		display.ClickEvent.Attach(this, &VanPointGUI::OnClick);
		Recompute();
	}

	void Recompute() {
		DLOG << "\n\nRecomputing\n\n";
		ImageRGB<byte> temp;
		int x_offs = 0;
		for (int i = 0; i < kfs.size(); i++) {
			ResizeImage(temp, kfs[i].image.nx()+100, kfs[i].image.ny()+100);
			temp.Clear(PixelRGB<byte>(0,0,0));
			//kfs[i].vpts.Compute(kfs[i].image);
			//kfs[i].vpts.DrawVanPointViz(temp);
			CopyImageInto(kfs[i].image.rgb, 50, 50, temp);

			Vec2D pt = fromToon(kfs[i].VptToImage(worldvpt));
			Vec2D impt(pt[0]+50, pt[1]+50);
			DrawSpot(temp, impt, PixelRGB<byte>(255,255,255), 6);

			CopyImageInto(temp, 0, x_offs, canvas);
			x_offs += temp.GetWidth();
		}
		display.RequestDraw();
	}

	void OnDraw(bool b) {
		if (display.GetWidth() != nx || display.GetHeight() != ny) {
			display.SetSize(nx, ny);
		}
		if (canvas.IsAlloced()) {
			DrawFunctions::Draw(display, canvas);
			display.Flush();
		}
	}

	void OnClick(int x, int y, int button) {
		for (int i = 0; i < kfs.size(); i++) {
			if (x < kfs[i].image.nx()+100) {
				int imx = x-50;
				int imy = y-50;

				worldvpt = kfs[i].VptToWorld(TooN::makeVector(imx, imy));

				for (int j = 0; j < kfs[i].vpts.lines.segments.size(); j++) {
					const LineSegment& seg = *kfs[i].vpts.lines.segments[j];
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
			}

			x -= kfs[i].image.nx()+100;
		}
		DLOG << x << " " << y;

		Recompute();
	}

	void OnKeyboard(unsigned char key, int x, int y) {
		switch (key) {
		case 'w':
			// Write out the current frame
			WriteImage("frame.png", canvas);
			cout << "Wrote frame.png\n";
			break;

		case 'r':
			Recompute();
			break;

		case 'v':
			ReloadVars();
			Recompute();
			break;

		case 'q':
		case 27: 
			GUI.StopParserThread();
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

	GUI.StartParserThread();

	// Load the keyframes
	ptr_vector<KeyFrame> kfs;
	for (int i = 1; i < argc; i++) {
		kfs.push_back(new KeyFrame);  // memory is now managed by the ptr_vector
		KeyFrame& kf = kfs.back();

		// Read image and unwarp
		ImageBundle raw(argv[i]);
		kf.unwarped.Compute(raw);
		ImageCopy(kf.unwarped.image, kf.image.rgb);

		// Load world-to-camera xform
		TooN::Vector<6> v;
		string imgfile(argv[i]);
		string posefile = imgfile.substr(0, imgfile.length()-4)+".info";
		cout << posefile << endl;
		ifstream input(posefile.c_str());
		input >> v;
		kf.CfW = TooN::SE3<>::exp(v);
		cout << kf.CfW << endl;
	}

	// Run the GUI
	VanPointGUI gui(kfs);
	gui.Run();

	return 0;
}
