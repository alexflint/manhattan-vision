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
#include <LU.h>

#include "misc.h"
#include "common_types.h"
#include "vanishing_points.h"
#include "timer.h"
#include "image_bundle.h"
#include "unwarped_image.h"
#include "map.h"
#include "vars.h"

#include "image_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;
using namespace VWEvents;
using namespace TooN;

class VanPointGUI : public EventHandler {
private:
	ImageRGB<byte> canvas;
	VW::DisplayGLUT display;
	int nx, ny;
	Map& map;
	VanishingPointDetector vpt_detector;
	vector<LineSegment> segments;  // line segments on the plane-at-infinity

public:
	VanPointGUI(Map& amap)
		: map(amap),
			display(1, 1, "Vanishing Points GUI") {
		nx = ny = 0;
		for (int i = 0; i < map.kfs.size(); i++) {
			nx += map.kfs[i].unwarped.image.nx()+100;
			ny = max(ny, map.kfs[i].unwarped.image.ny()+100);
		}
		canvas.AllocImageData(nx, ny);

		display.DrawEvent.Attach(this, &VanPointGUI::OnDraw);
		display.KeyboardEvent.Attach(this, &VanPointGUI::OnKeyboard);
	}

	// Detect lines and add to global list
	void DetectLinesAndBootstrap() {
		map.DetectLines();
	}

	// Propagate the vanishing points to the source images
	void PropagateVanPts() {
		map.PropagateVanPts();
	}

	void Recompute() {
		map.DetectLines();
		map.ComputeVanPts();
		map.PropagateVanPts();
	}

	void Redraw() {
		ImageRGB<byte> temp;
		int x_offs = 0;
		ImageRef padding(50,50);
		for (int i = 0; i < map.kfs.size(); i++) {
			KeyFrame& kf = map.kfs[i];
			ImageRef sz = kf.unwarped.image.sz();
			ResizeImage(temp, sz+padding*2);

			// Draw vanishing points and lines
			kf.vpts.DrawVanPointViz(temp);

			// Draw points
			Vector<2> offs = TooN::makeVector(sz.x+padding.x+x_offs, sz.y+padding.y);
			Matrix<3,4> proj = TooN::Zeros;
			proj.slice<0,0,3,3>() = TooN::Identity;
			Matrix<3,4> world_to_image = kf.unwarped.retina_to_image * proj * kf.CfW;
			BOOST_FOREACH(const Vector<3>& worldpt, map.pts) {
				Vector<3> retinapt = kf.CfW * worldpt;
				Vector<2> imagept = HProj(kf.unwarped.retina_to_image * retinapt);
				Vector<2> pos = makeVector(roundi(imagept[0]+padding.x), roundi(imagept[1]+padding.y));
				if (pos[0] >= 0 && pos[0] < sz.x && pos[1] >= 0 && pos[1] < sz.y) {
					DrawSpot(temp, pos, BrightColors::Get(5), 1);
				}
			}

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
			VW::DrawFunctions::Draw(display, canvas);
			display.Flush();
		}
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
			Redraw();
			break;

		case 'd':
			DetectLinesAndBootstrap();
			vpt_detector.StoreVanPts();
			PropagateVanPts();
			Redraw();
			break;

		case 'e':
			vpt_detector.EStep();
			vpt_detector.StoreVanPts();
			PropagateVanPts();
			Redraw();
			break;

		case 'm':
			vpt_detector.MStep();
			vpt_detector.StoreVanPts();
			PropagateVanPts();
			Redraw();
			break;

		case 'v':
			ReloadVars();
			Recompute();
			Redraw();
			break;

		case 'q':
		case 27: 
			GVars3::GUI.StopParserThread();
			exit(0);
			break;
		}
	}

	void Run() {
		VW::GLUTLoop();
	}
};


int main(int argc, char **argv) {
	VW::GLUTInit(argc, argv);
	InitVars(argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map
	Map map;
	map.Load("Map");

	// Run the GUI
	VanPointGUI gui(map);
	gui.Run();

	return 0;
}
