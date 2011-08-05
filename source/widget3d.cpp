#include <GL/glut.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include <LU.h>

#include "widget3d.h"
#include "viewer3d.h"
#include "common_types.h"
#include "geom_utils.h"
#include "image_utils.h"

#include "gl_utils.tpp"

namespace indoor_context {
	using namespace toon;

	// Wraps a function object as a Widget
	class FunctionWidget : public Widget3D {
	public:
		function<void()> f;
		FunctionWidget(function<void()> ff) : f(ff) { }
		virtual void OnRender() { f(); }
	};

	////////////////////////////////////////////////////////
	// Widget3D
	Widget3D::Widget3D()
	: _viewer(NULL),
		_hovering(false),
		_selectable(true),
		_selected(false),
		_visible(true),
		_draggable(false),
		_zorder(0) {
	}

	void Widget3D::Traverse(const function<void(Widget3D*)>& f) {
		f(this);
		BOOST_FOREACH(Widget3D* w, children()) {
			w->Traverse(f);
		}
	}

	void Widget3D::Invalidate() {
		if (attached()) {
			viewer().Invalidate();
		}
	}

	void Widget3D::SetVisible(bool v) {
		_visible = v;
		Invalidate();
		VisibilityChanged.fire();
	}

	void Widget3D::SetSelected(bool v) {
		// We don't check that selected_ != v, but Viewer3D::Select does
		// something similar (equally efficient but more general)
		if (v) {
			viewer().Select(*this);
		} else {
			viewer().Deselect();
		}
	}

	Widget3D& Widget3D::Add(Widget3D& w, char toggleKey) {
		_children.push_back(&w);
		if (attached()) {
			w.NotifyAttached(_viewer);
		}
		if (toggleKey != 0) {
			viewer().window().KeyStroke(toggleKey).add
				(bind(&Widget3D::ToggleVisible, ref(w)));
		}
		return w;
	}

	Widget3D& Widget3D::AddOwned(Widget3D* w, char toggleKey) {
		_owned_children.push_back(w);
		return Add(*w, toggleKey);
	}

	Widget3D& Widget3D::Add(boost::function<void()> f, char toggleKey) {
		return AddOwned(new FunctionWidget(f), toggleKey);
	}

	void Widget3D::RemoveChild(int i) {
		_children[i]->NotifyDeattached();
		_children.erase(_children.begin()+i);
		// TODO: figure out a way to find references to this child within
		// key_handlers (this is difficult because we just have some
		// function pointers!) After that's done, also delete the child
		// from owned_children_. As it is, we never actually free any
		// memory (uh oh).
	}

	void Widget3D::ClearChildren() {
		for (int i = children().size()-1; i >= 0; i--) {
			RemoveChild(i);
		}
	}

	bool Widget3D::HitTest(const Vec2& mouse) const {
		return false;
	}

	Widget3D* Widget3D::GetWidgetAt(const Vec2& mouse) {
		if (!visible()) return NULL;

		Widget3D* hit = HitTest(mouse) ? this : NULL;
		BOOST_FOREACH(Widget3D* w, children()) {
			Widget3D* cand = w->GetWidgetAt(mouse);
			if (cand != NULL && (hit == NULL || cand->zorder() < hit->zorder())) {
				hit = cand;
			}
		}
		return hit;
	}




	void Widget3D::NotifyPreRender() {
		OnPreRender();
		BOOST_FOREACH(Widget3D* w, children()) {
			w->NotifyPreRender();
		}
		PreRender.fire();
	}

	void Widget3D::NotifyRender() {
		if (visible()) {
			OnRender();
			BOOST_FOREACH(Widget3D* w, children()) {
				w->NotifyRender();
			}
			Render.fire();
		}
	}

	void Widget3D::NotifyMouseDown(int button, const Vec2& mouse) {
		if (selectable()) {
			SetSelected(true);
		}
		MouseDown.fire();
	}

	void Widget3D::NotifyMouseUp(int button, const Vec2& mouse) {
		MouseUp.fire();
	}

	void Widget3D::NotifyMouseMove(const Vec2& mouse) {
		MouseMove.fire();
	}

	void Widget3D::NotifyMouseDrag(int button, const Vec2& mouse) {
		MouseDrag.fire();
	}

	void Widget3D::NotifyClick(int button, const Vec2& mouse) {
		OnClick(button, mouse);
		Click.fire();
	}

	void Widget3D::NotifyDoubleClick(int button, const Vec2& mouse) {
		OnDoubleClick(button, mouse);
		DoubleClick.fire();
	}

	void Widget3D::NotifySelectedChanged(bool v) {
		_selected = v;
		SelectedChanged.fire();
	}

	void Widget3D::NotifyHoverChanged(bool v) {
		_hovering = v;
		HoverChanged.fire();
	}

	void Widget3D::NotifyAttached(Viewer3D* viewer) {
		_viewer = viewer;
		BOOST_FOREACH(Widget3D* w, children()) {
			w->NotifyAttached(viewer);
		}
		Attached.fire();
	}

	void Widget3D::NotifyDeattached() {
		_viewer = NULL;
		Deattached.fire();
	}





	void PointWidget::OnRender() {
		glColorP(color);
		glPointSize(size);
		WITH(GL_POLYGON_OFFSET_LINE) {
			glPolygonOffset(0, depth_offset);
			GL_PRIMITIVE(GL_POINTS) {
				glVertexV(p);
			}
		}
	}



	////////////////////////////////////////////////////////
	// QuadWidget
	QuadWidget::QuadWidget()
		: boundary(makeVector(0,0,0),
							 makeVector(0,0,0),
							 makeVector(0,0,0),
							 makeVector(0,0,0)),
			color(255,255,0),
			border_width(1.2) {
	}

	QuadWidget::QuadWidget(const Polygon<4>& polygon)
		: boundary(polygon),
			color(255,255,0),
			border_width(1.2)	{
	}

	QuadWidget::QuadWidget(const Matrix<3,2>& m,
												 float left,
												 float right,
												 float top,
												 float bottom)
		: boundary(m*makeVector(left,top),
							 m*makeVector(right,top),
							 m*makeVector(right,bottom),
							 m*makeVector(left,bottom)),
			color(255,255,0),
			border_width(1.2) {
	}

	void QuadWidget::OnRender() {
		if (border_width > 0) {
			glColorP(color);
			glLineWidth(border_width);
			GL_PRIMITIVE(GL_LINE_LOOP) {
				for (int i = 0; i < 4; i++) {
					glVertexV(boundary.verts[i]);
				}
			}
		}

		glColor4ub(color.r, color.g, color.b, 16);
		glDepthMask(GL_FALSE);
		WITH(GL_BLEND) GL_PRIMITIVE(GL_QUADS) {
			for (int i = 0; i < 4; i++) {
				glVertexV(boundary.verts[i]);
			}
		}
		glDepthMask(GL_TRUE);
	}

	LineWidget& LineWidgetBloc::AddNewMember() {
		LineWidget* w = new LineWidget;
		w->width = width;
		w->depthOffset = depth_offset;
		w->bloc = this;
		AddOwned(w);
		members.push_back(w);
		return *w;
	}

	LineWidget& LineWidgetBloc::AddNewMember(const Vec3& start,
																					 const Vec3& end,
																					 const PixelRGB<byte>& color=Colors::black()) {
		LineWidget* w = new LineWidget(start, end, width, color);
		w->depthOffset = depth_offset;
		w->bloc = this;
		AddOwned(w);
		members.push_back(w);
		return *w;
	}

	void LineWidgetBloc::OnRender() {
		// Set up line parameters
		glLineWidth(width);

		// Draw the widgets
		int n = 0;
		glError();
		WITH(GL_POLYGON_OFFSET_LINE) {
			glPolygonOffset(0, depth_offset);
			GL_PRIMITIVE(GL_LINES) {
				for (int i = 0; i < members.size(); i++) {
					if (members[i]->subsumed) {
						n++;
						glColorP(members[i]->color);
						glVertexV(members[i]->lineseg.start);
						glVertexV(members[i]->lineseg.end);
					}
				}
			}
		}

		// Get their screen positions
		if (selectable()) {
			GluProjector proj(viewer().window().size());
			BOOST_FOREACH(LineWidget* w, members) {
				w->screen_line.start = unproject(proj.Project(w->lineseg.start));
				w->screen_line.end = unproject(proj.Project(w->lineseg.end));
			}
		}
	}






	LineWidget::LineWidget()
	: width(1.2),
		color(1,1,1),
		selectColor(Colors::white()),
		depthOffset(0),
		subsumed(false),
		bloc(NULL) {
	}

	LineWidget::LineWidget(const Vec3& start,
												 const Vec3& end,
												 float w,
												 const PixelRGB<byte>& c)
	: lineseg(start, end),
		width(w),
		color(c),
		selectColor(Colors::white()),
		depthOffset(0),
		subsumed(false),
		bloc(NULL) {
	}

	LineWidget::LineWidget(const LineSeg& seg,
												 float w,
												 const PixelRGB<byte>& c)
	: lineseg(seg),
		width(w),
		color(c),
		selectColor(Colors::white()),
		depthOffset(0),
		subsumed(false),
		bloc(NULL) {
	}

	bool LineWidget::HitTest(const Vec2& mouse) const {
		if (selectable()) {
			Vec2 sa = project(screen_line.start);
			Vec2 sb = project(screen_line.end);
			Vec2 smid = (sa+sb)/2.0;
			Vec2 sline = makeVector(sa[1]-sb[1], sb[0]-sa[0]) / norm(sb-sa);
			double slen_sq = norm_sq(sa-sb);
			double d = abs((mouse-sa)*sline);
			return d < kHitThresh*width && norm_sq(mouse-smid) <= slen_sq/2.0;
		} else {
			return false;
		}
	}

	void LineWidget::OnPreRender() {
		subsumed = (bloc != NULL
								&& visible() && !hovering() && !selected()
								&& width == bloc->width
								&& depthOffset == bloc->depth_offset);
	}

	void LineWidget::OnRender() {
		if (subsumed) return;

		// Line width
		if (hovering() || selected()) {
			glLineWidth(width*2);
		} else {
			glLineWidth(width);
		}

		// Color
		if (selected()) {
			glColorP(selectColor);
		} else {
			glColorP(color);
		}

		// Depth test
		if (hovering()) {
			glDisable(GL_DEPTH_TEST);
		}

		glEnable(GL_POLYGON_OFFSET_LINE);
		glPolygonOffset(0, depthOffset);
		GL_PRIMITIVE(GL_LINES) {
			glVertexV(lineseg.start);
			glVertexV(lineseg.end);
		}
		glDisable(GL_POLYGON_OFFSET_LINE);


		if (hovering()) {
			glEnable(GL_DEPTH_TEST);
		}

		if (selectable()) {
			screen_line.start = unproject(viewer().ProjectToScreen(lineseg.start));
			screen_line.end = unproject(viewer().ProjectToScreen(lineseg.end));
		}
	}



	void GroundPlaneWidget::OnRender() {
		glLineWidth(1.2);
		glColor3f(0.4,0.4,0.4);
		GL_PRIMITIVE(GL_LINES) {
			for (int i = -5; i <= 5; i++) {
				glVertex2f(i, -5);
				glVertex2f(i, 5);
				glVertex2f(-5, i);
				glVertex2f(5, i);
			}
		}
	}

}
