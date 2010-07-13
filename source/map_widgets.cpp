#include "map_widgets.h"

#include <boost/format.hpp>

#include <GL/gl.h>
#include <GL/glut.h>

#include "viewer3d.h"
#include "common_types.h"
#include "vars.h"
#include "map.h"
#include "gl_utils.tpp"
#include "geom_utils.h"
#include "lazyvar.h"

#include "math_utils.tpp"
#include "range_utils.tpp"
#include "io_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
using namespace toon;

const double kDefaultRetinaZ = 0.5;
lazyvar<double> gvRetinaPosDelta("MapViz.RetinaPosDelta");

/////////////////// KeyFrameWidget ////////////////////

KeyFrameWidget::KeyFrameWidget(const KeyFrame& k, double ret_z)
: retina_z(ret_z),
  kf(k),
  line_bloc(1.2, -5),
  guided_line_bloc(1.2, -5),
  border_width(1.2),
  border_color(127, 127, 127),
  line_state(0),  // no lines
  show_textons(false) {
	Add(line_bloc);
	Add(guided_line_bloc);
}

bool KeyFrameWidget::HitTest(const Vector<2>& mouse) const {
	return PointInQuad(pa,pb,pc,pd,mouse);
}

void KeyFrameWidget::GLTransformToCameraCoords() const {
	const SE3<>& inv = kf.pc->invpose;
	Matrix<4> invmat = Identity;
	invmat.slice<0,0,3,3>() = inv.get_rotation().get_matrix();
	invmat.slice<0,3,3,1>() = inv.get_translation().as_col();
	TransformGL(invmat);
}

void KeyFrameWidget::OnRender() {
	GL_MATRIX_SCOPE {
		GLTransformToCameraCoords();

		// Bind this keyframe into the current GL texture. On the first
		// invokation, this will load the texture into graphics memory.
		kf.unwarped.image.BindGLTexture();

		// Project the quad boundary to the screen
		const Vector<2>& tl = kf.unwarped.TLinRetina();
		const Vector<2>& tr = kf.unwarped.TRinRetina();
		const Vector<2>& bl = kf.unwarped.BLinRetina();
		const Vector<2>& br = kf.unwarped.BRinRetina();
		pa = viewer().ProjectToScreen(unproject(tl)*retina_z);
		pb = viewer().ProjectToScreen(unproject(tr)*retina_z);
		pc = viewer().ProjectToScreen(unproject(br)*retina_z);
		pd = viewer().ProjectToScreen(unproject(bl)*retina_z);

		// Draw the image
		WITHOUT(GL_BLEND) WITH(GL_TEXTURE_2D) GL_PRIMITIVE(GL_QUADS) {
			glTexCoord2f(0,0);
			glVertex4f(tl[0], tl[1], 1, 1.0/retina_z);
			glTexCoord2f(0,1);
			glVertex4f(tl[0], br[1], 1, 1.0/retina_z);
			glTexCoord2f(1,1);
			glVertex4f(br[0], br[1], 1, 1.0/retina_z);
			glTexCoord2f(1,0);
			glVertex4f(br[0], tl[1], 1, 1.0/retina_z);
		}

		// Draw the image border
		if (hovering()) {
			glDisable(GL_DEPTH_TEST);
			glLineWidth(2);
			glColor3f(0, 0, 1);
		} else if (selected()) {
			glLineWidth(2);
			glColor3f(0, 1, 0);
		} else {
			glLineWidth(border_width);
			glColorP(border_color);
		}
		GL_PRIMITIVE(GL_LINE_LOOP) {
			glVertex4f(tl[0], tl[1], 1, 1.0/retina_z);
			glVertex4f(tl[0], br[1], 1, 1.0/retina_z);
			glVertex4f(br[0], br[1], 1, 1.0/retina_z);
			glVertex4f(br[0], tl[1], 1, 1.0/retina_z);
		}
		if (hovering()) {
			glEnable(GL_DEPTH_TEST);
		}

		// Draw the camera center
		glColor3f(1,0,0);
		glPointSize(3);
		GL_PRIMITIVE(GL_POINTS) {
			glVertex3f(0,0,0);
		}
		glError();

		// Draw the frustum
		if (selected()) {
			glLineWidth(0.5);
			glColor3f(1,0,0);
			GL_PRIMITIVE(GL_LINES) {
				for (int i = 0; i < 4; i++) {
					double fx = (i&1) ? tl[0] : br[0];
					double fy = (i&2) ? tl[1] : br[1];
					glVertex2f(0,0);
					glVertex4f(fx, fy, 1, 0.1);
				}
			}
			glError();
		}
	}
}

Vector<3> KeyFrameWidget::ImagePtToWorld(const Vector<2>& p) {
	// Pixel Coordinates -> Homogeneous Pixel Coords -> Retina Plane (Z=1) Coordates
	//  -> (Z = retina_z) Plane Coordinates -> World Coordaintes -> whew!
	const Matrix<3>& im_to_ret = kf.unwarped.image_to_retina;
	return kf.pc->invpose * (retina_z * atretina(im_to_ret * unproject(p)));
}

Vector<3> KeyFrameWidget::WorldToRetina(const Vector<3>& p) {
	return kf.pc->invpose * (retina_z * atretina(kf.pc->pose * p));
}

LineWidget& KeyFrameWidget::AddLineInRetina(const Vector<3>& ret_a,
                                            const Vector<3>& ret_b,
                                            const float width,
                                            const PixelRGB<byte>& color) {
	LineWidget* w = new LineWidget(kf.pc->invpose * (retina_z * atretina(ret_a)),
			kf.pc->invpose * (retina_z * atretina(ret_b)),
			width,
			color);
	AddOwned(w);
	return *w;
}

void KeyFrameWidget::Line_Click(int index, const string& label) {
	DLOG << label;
}

void KeyFrameWidget::ConfigureLineWidget(LineWidget& w,
                                         const LineDetection& det,
                                         int index,
                                         bool visible,
                                         bool add_events,
                                         const string& label) {
	w.lineseg.start = ImagePtToWorld(project(det.seg.start));
	w.lineseg.end = ImagePtToWorld(project(det.seg.end));
	w.SetZorder(-1);
	w.SetVisible(visible);
	w.color = (det.axis == -1 ?
			Colors::black() : Colors::primary(det.axis));
	if (add_events) {
		w.Click.add(bind(&KeyFrameWidget::Line_Click, this, index, label));
	}
}

// private
void KeyFrameWidget::ConfigureLineWidgets() {
	// Configure widgets for canny lines
	bool persist = (line_widgets.size() == kf.line_detector.detections.size());
	// This is not for efficiency, but so that we preserve event
	// handlers associated with the lines segments from outside
	if (!persist) {
		line_widgets.clear();
	}
	COUNTED_FOREACH(int i, const LineDetection& det, kf.line_detector.detections) {
		if (!persist) {
			line_widgets.push_back(&line_bloc.AddNewMember());
		}
		string label = str(format("Canny line clicked: axis=%d, index=%d") % det.axis % i);
		ConfigureLineWidget(*line_widgets[i], det, i, line_state==1, !persist, label);
	}


	// Configure widgets for guided lines
	const vector<LineDetection>* dets = kf.guided_line_detector.detections;
	// This is not for efficiency, but so that we preserve event
	// handlers associated with the lines segments from outside
	persist = guided_line_widgets.size() == dets[0].size()+dets[1].size()+dets[2].size();
	if (!persist) {
		guided_line_widgets.clear();
	}
	int i = 0;
	for (int axis = 0; axis < 3; axis++) {
		COUNTED_FOREACH(int j, const LineDetection& det, dets[axis]) {
			if (!persist) {
				guided_line_widgets.push_back(&guided_line_bloc.AddNewMember());
			}
			string label = str(format("Guided line clicked: axis=%d, index=%d") % axis % j);
			ConfigureLineWidget(*guided_line_widgets[i], det, j, line_state==2, !persist, label);
			i++;
		}
	}
}

void KeyFrameWidget::OnClick(int button, const Vector<2>& mouse) {
	DLOG << "Keyframe " << kf.id << " clicked.\n  Filename:" << kf.image_file;
}

void KeyFrameWidget::OnDoubleClick(int button, const Vector<2>& mouse) {
	viewer().viewCentre = -kf.pc->invpose.get_translation();
}





////////////////// PointCloudWidget //////////////////////

PointCloudWidget::PointCloudWidget(const vector<Vec3>& pts)
: points(pts), selected_point(-1) {
}

void PointCloudWidget::OnRender() {
	if (viewer().isAntialiasing) return;  // don't antialias the points

	// Draw the points
	glPointSize(2.0);
	glColor3f(0,1,0);
	GL_PRIMITIVE(GL_POINTS) {
		COUNTED_FOREACH(int i, const Vector<3>& v, points) {
			if (i != selected_point) {
				glVertexV(v);
			}
		}
	}

	// Draw the selected point
	if (selectable() && selected_point != -1) {
		glPointSize(8.0);
		glColor3f(0.5, 0.8, 0.8);
		GL_PRIMITIVE(GL_POINTS) {
			glVertexV(points[selected_point]);
		}
	}

	// Record the screen projections
	if (selectable()) {
		screen_pts.resize(points.size());
		COUNTED_FOREACH(int i, const Vector<3>& v, points) {
			screen_pts[i] = viewer().ProjectToScreen(v);
		}
	}
}

int PointCloudWidget::GetPointAt(const Vector<2>& mouse) const {
	static const double kHitMargin = 4.0;
	int best_i = -1;
	double best_dsq = kHitMargin*kHitMargin;
	COUNTED_FOREACH(int i, const Vector<2>& v, screen_pts) {
		double dsq = norm_sq(mouse-v);
		if (dsq < best_dsq) {
			best_i = i;
			best_dsq = dsq;
		}
	}
	return best_i;
}

bool PointCloudWidget::HitTest(const Vector<2>& mouse) const {
	return GetPointAt(mouse) != -1;
}

void PointCloudWidget::OnClick(int button, const Vector<2>& mouse) {
	selected_point = GetPointAt(mouse);
	SelectedPointChanged.fire();
}

///////////////////// MapWidget /////////////////////////////
MapWidget::MapWidget() {
}

MapWidget::MapWidget(const Map* themap) {
	Configure(themap);
}

void MapWidget::Configure(const Map* themap) {
	map = themap;

	// Setup child components
	ground_plane = new GroundPlaneWidget();
	AddOwned(ground_plane);
	pts_widget = new PointCloudWidget(map->pts);
	pts_widget->SetSelectable(false);
	AddOwned(pts_widget);
	BOOST_FOREACH (const KeyFrame& kf, map->kfs) {
		RegisterKeyFrame(kf);
	}

	// Bind keys
	if (attached()) {
		BindKeys();
	} else {
		Attached.add(bind(&MapWidget::BindKeys, this));
	}
}

KeyFrameWidget& MapWidget::RegisterKeyFrame(const KeyFrame& kf) {
	KeyFrameWidget* kfw = new KeyFrameWidget(kf, kDefaultRetinaZ);
	kf_widgets.push_back(kfw);
	AddOwned(kfw);
	return *kfw;
}

void MapWidget::BindKeys() {
	viewer().window().KeyStroke('p').add(bind(&PointCloudWidget::ToggleVisible, ref(pts_widget)));
	viewer().window().KeyStroke(',').add(bind(&MapWidget::ChangeRetinaPos, this, -1));
	viewer().window().KeyStroke('.').add(bind(&MapWidget::ChangeRetinaPos, this, 1));
	viewer().window().KeyStroke('l').add(bind(&MapWidget::ToggleLines, this));
}

void MapWidget::SetRetinaPos(double z) {
	BOOST_FOREACH(KeyFrameWidget* kfwidget, kf_widgets) {
		kfwidget->retina_z = z;
		if (kfwidget->line_state != 0) {
			kfwidget->ConfigureLineWidgets();
		}
	}
	Invalidate();
}

void MapWidget::ChangeRetinaPos(int delta) {
	SetRetinaPos(kf_widgets[0]->retina_z * pow(*gvRetinaPosDelta, delta));
}

void MapWidget::ToggleLines() {
	// It could be that some keyframes have lines visible while others
	// are not, but the most sensible thing to do here is to force
	// them all to be the same.
	int line_state = (kf_widgets[0]->line_state + 1)%3;
	BOOST_FOREACH(KeyFrameWidget* kfwidget, kf_widgets) {
		kfwidget->line_state = line_state;
		kfwidget->ConfigureLineWidgets();
	}
}

}
