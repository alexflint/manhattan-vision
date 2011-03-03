#include "floorplan_editor.h"

#include <boost/foreach.hpp>

#include "map.h"
#include "map_widgets.h"
#include "glut_window.h"
#include "map.pb.h"
#include "geom_utils.h"

#include "counted_foreach.tpp"
#include "gl_utils.tpp"
//#include "numeric_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	const double kHitMargin = 0.1;  // in canvas coordinates

	FloorPlanEditor::FloorPlanEditor(const Map& themap)
		: camViewState(ViewType::WIREFRAME),
			map(themap),
			camView("Camera View"),
			filename_("floorplan.pro") {
		ResetInternal();
		SetUIState(UIState::INSERT);
		
		mapWidget.Configure(&map);
		DREPORT(map.pts.size())
		mapWidget.PreRender.add(bind(&FloorPlanEditor::mapWidget_PreRender, this));

		planView.viewOrtho = true;
		planView.window().SetTitle("Plan View");
		planView.window().MouseMove.add(bind(&FloorPlanEditor::planView_MouseMove, this, _1));
		planView.window().MouseDown.add(bind(&FloorPlanEditor::planView_MouseDown, this, _1, _2));
		planView.window().MouseDrag.add(bind(&FloorPlanEditor::planView_MouseDrag, this, _1, _2));
		planView.window().KeyStroke('e').add(bind(&FloorPlanEditor::ToggleUIState, this));
		planView.window().KeyStroke('`').add(bind(&FloorPlanEditor::ResetView, this));
		planView.Add(mapWidget);
		planView.Add(bind(&FloorPlanEditor::RenderPlanView, this));

		sideView.viewOrtho = true;
		//sideView.SetNavigable(false);
		sideView.window().SetTitle("Side View");
		sideView.window().MouseMove.add(bind(&FloorPlanEditor::sideView_MouseMove, this, _1));
		sideView.window().MouseDown.add(bind(&FloorPlanEditor::sideView_MouseDown, this, _1, _2));
		sideView.window().MouseUp.add(bind(&FloorPlanEditor::sideView_MouseUp, this, _1, _2));
		sideView.window().MouseDrag.add(bind(&FloorPlanEditor::sideView_MouseDrag, this, _1, _2));
		sideView.window().SetPosition(ImageRef(0, planView.window().size().y+20));
		sideView.Add(mapWidget);
		sideView.Add(bind(&FloorPlanEditor::RenderSideView, this));

		camView.SetPosition(ImageRef(planView.window().size().x+20, 0));
		camView.Display.add(bind(&FloorPlanEditor::camView_Display, this));

		freeView.window().SetTitle("Side View");
		freeView.window().SetPosition(planView.window().size() + ImageRef(20,20));
		freeView.Add(mapWidget);
		freeView.Add(bind(&FloorPlanEditor::RenderFreeView, this));

		planView.window().KeyDown.add(bind(&FloorPlanEditor::anyView_KeyDown, this, _1));
		sideView.window().KeyDown.add(bind(&FloorPlanEditor::anyView_KeyDown, this, _1));
		freeView.window().KeyDown.add(bind(&FloorPlanEditor::anyView_KeyDown, this, _1));
		camView.KeyDown.add(bind(&FloorPlanEditor::anyView_KeyDown, this, _1));

		planView.window().SpecialKeyDown.add(bind(&FloorPlanEditor::anyView_SpecialKeyDown, this, _1));
		sideView.window().SpecialKeyDown.add(bind(&FloorPlanEditor::anyView_SpecialKeyDown, this, _1));
		camView.SpecialKeyDown.add(bind(&FloorPlanEditor::anyView_SpecialKeyDown, this, _1));
	}

	void FloorPlanEditor::ResetInternal() {
		selectedFrameIndex_ = 0;
		mouseState = 0;
		hoverItem = -1;
		selectedItem = -1;
		enableSnap = true;
		ResetView();
	}

	void FloorPlanEditor::Run() {
		camView.Create();
		sideView.Create();
		freeView.Create();
		planView.Create(); // put this last so that it will be selected by default
		GlutWindow::Loop();
	}

	void FloorPlanEditor::Invalidate() {
		planView.Invalidate();
		sideView.Invalidate();
		freeView.Invalidate();
		camView.Invalidate();
	}

	void FloorPlanEditor::ResetView() {
		planView.viewRotation = Zeros;
		sideView.viewRotation = makeVector(0,-90,0);
		planView.Invalidate();
		sideView.Invalidate();
	}

	void FloorPlanEditor::ToggleCamViewState() {
		camViewState = (camViewState+1)%2;
		camView.Invalidate();
	}

	void FloorPlanEditor::ToggleUIState() {
		SetUIState((uiState+1)%3);
	}

	void FloorPlanEditor::SetUIState(int state) {
		uiState = state;
		switch (uiState) {
		case UIState::VIEW:
			planView.SetNavigable(true);
			planView.window().SetCursor(GLUT_CURSOR_LEFT_ARROW);
			break;
		case UIState::INSERT:
			planView.SetNavigable(false);
			planView.window().SetCursor(GLUT_CURSOR_CROSSHAIR);
			break;
		case UIState::MODIFY:
			planView.SetNavigable(false);
			planView.window().SetCursor(GLUT_CURSOR_INFO);
			break;
		}
	}

	Vector<2> FloorPlanEditor::MouseToCanvas(const Vector<2>& mousePt) {
		return planView.MouseToPlane(mousePt, makeVector(0,0,1,0)).slice<0,2>();
	}

	Vector<2> FloorPlanEditor::Snap(const Vector<2>& v, int snap_axis, int ignore) {
		const double kSnapMargin = 0.2;  // in canvas coordinates

		double dmin = kSnapMargin;
		double snap = v[snap_axis];
		COUNTED_FOREACH(int i, const proto::Vec2& vert, floorplan->vertices()) {
			Vector<2> u = asToon(vert);
			if (i == ignore || i == ignore+1) continue;
			double d = abs(u[snap_axis]-v[snap_axis]);
			if (d < dmin) {
				dmin = d;
				snap = u[snap_axis];
			}
		}

		Vector<2> snapped = v;
		snapped[snap_axis] = snap;
		return snapped;
	}

	void FloorPlanEditor::planView_MouseDown(int button, const Vector<2>& mousePt) {
		static const double kNan = numeric_limits<double>::quiet_NaN();
		if (uiState == UIState::INSERT) {
			mouseState = 1;  // means clicked but no move yet
			int n = floorplan->vertices_size();
			if (n==0 || isnan(floorplan->vertices(n-1).x1())) {
				*floorplan->add_vertices() = asProto(MouseToCanvas(mousePt));
			} else {
				*floorplan->add_vertices() = asProto(endPt);
				for (int i = n-2; i >= 0 && !isnan(floorplan->vertices(i).x1()); i--) {
					if (endPt == asToon(floorplan->vertices(i))) {
						if (i == n-2) {
							// remove the newly added vertex for double-clicks
							floorplan->mutable_vertices()->RemoveLast();
						}
						*floorplan->add_vertices() = asProto(makeVector(kNan, kNan));
						mouseState = 0;
						break;
					}
				}
			}
			planView.Invalidate();
			camView.Invalidate();

		} else if (uiState == UIState::MODIFY) {
			selectedItem = hoverItem;
			if (hoverItem != -1) {
				double dx = floorplan->vertices(hoverItem).x1() - floorplan->vertices(hoverItem+1).x1();
				double dy = floorplan->vertices(hoverItem).x2() - floorplan->vertices(hoverItem+1).x2();
				dragAxis = abs(dx) < abs(dy) ? 0 : 1;
				Vector<2> canvasPt = MouseToCanvas(mousePt);
				Vector<2> hoverPt = asToon(floorplan->vertices(hoverItem));
				dragOffset = hoverPt[dragAxis] - canvasPt[dragAxis];
			}
		}
	}

	void FloorPlanEditor::planView_MouseDrag(int button, const Vector<2>& mousePt) {
		if (uiState == UIState::MODIFY && selectedItem != -1) {
			Vector<2> dragPt = MouseToCanvas(mousePt);
			dragPt[dragAxis] += dragOffset;
			if (enableSnap) {
				dragPt = Snap(dragPt, dragAxis, selectedItem);
			}
			if (dragAxis == 0) {
				floorplan->mutable_vertices(selectedItem)->set_x1(dragPt[0]);
				floorplan->mutable_vertices(selectedItem+1)->set_x1(dragPt[0]);
			} else {
				floorplan->mutable_vertices(selectedItem)->set_x2(dragPt[1]);
				floorplan->mutable_vertices(selectedItem+1)->set_x2(dragPt[1]);
			}
			planView.Invalidate();
			camView.Invalidate();
		}
	}

	void FloorPlanEditor::planView_MouseMove(const Vector<2>& mousePt) {
		if (uiState == UIState::INSERT) {
			if (mouseState == 1) {
				mouseState = 2; // means hovering
			}
			hoverPt = MouseToCanvas(mousePt);
			if (mouseState == 2) {
				Vector<2> ref = asToon(floorplan->vertices(floorplan->vertices_size()-1));
				int draw_axis = (abs(hoverPt[0]-ref[0]) > abs(hoverPt[1]-ref[1])) ? 0 : 1;
				if (enableSnap) {
					endPt = Snap(hoverPt, draw_axis);
				}
				endPt[1-draw_axis] = ref[1-draw_axis];
			}

		} else if (uiState == UIState::MODIFY) {
			Vector<2> planePt = MouseToCanvas(mousePt);
			hoverItem = -1;
			for (int i = 0; i+1 < floorplan->vertices_size(); i++) {
				Vector<2> a = asToon(floorplan->vertices(i));
				Vector<2> b = asToon(floorplan->vertices(i+1));
				if (!isnan(a[0]) && !isnan(b[0]) &&
						GetLineSegDistance(a, b, planePt) < kHitMargin) {
					hoverItem = i;
					break;
				}
			}
		}

		planView.Invalidate();
		camView.Invalidate();
	}






	double FloorPlanEditor::MouseToHeight(const Vector<2>& mousePt) {
		return sideView.MouseToPlane(mousePt, makeVector(0,1,0,0))[2];
	}

	void FloorPlanEditor::sideView_MouseDown(int button, const Vector<2>& mousePt) {
		double z = MouseToHeight(mousePt);
		if (sideHoverItem == 1) {
			sideDragOffset = z-floorplan->zfloor();
		} else if (sideHoverItem == 2) {
			sideDragOffset = z-floorplan->zceil();
		}
		sideDragItem = sideHoverItem;
		sideView.Invalidate();
		camView.Invalidate();
	}

	void FloorPlanEditor::sideView_MouseUp(int button, const Vector<2>& mousePt) {
		if (sideDragItem == 1) {
			DLOG << "Floor plane set to z=" << floorplan->zfloor();
		} else if (sideDragItem == 2) {
			DLOG << "Ceiling plane set to z=" << floorplan->zceil();
		}

		sideDragItem = 0;
		sideView.Invalidate();
		camView.Invalidate();
	}

	void FloorPlanEditor::sideView_MouseDrag(int button, const Vector<2>& mousePt) {
		double z = MouseToHeight(mousePt);
		if (sideDragItem == 1) {
			floorplan->set_zfloor(z-sideDragOffset);
		} else if (sideDragItem == 2) {
			floorplan->set_zceil(z-sideDragOffset);
		}
		sideView.Invalidate();
		camView.Invalidate();
	}

	void FloorPlanEditor::sideView_MouseMove(const Vector<2>& mousePt) {
		double z = MouseToHeight(mousePt);
		if (abs(z-floorplan->zfloor()) < kHitMargin) {
			sideHoverItem = 1;
			sideView.window().SetCursor(GLUT_CURSOR_BOTTOM_SIDE);
		} else if (abs(z-floorplan->zceil()) < kHitMargin) {
			sideHoverItem = 2;
			sideView.window().SetCursor(GLUT_CURSOR_TOP_SIDE);
		} else {
			sideHoverItem = 0;
			sideView.window().SetCursor(GLUT_CURSOR_LEFT_ARROW);
		}
		sideView.Invalidate();
		camView.Invalidate();
	}


	void FloorPlanEditor::camView_Display() {
		if (camViewState == ViewType::WIREFRAME) {
			glClearColor(0,0,0,0);
		} else {
			glClearColor(0,0,1,0);
		}
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		CHECK_LT(selectedFrameIndex_, map.kfs.size())
			<< "There are only " << map.kfs.size() << " keyframes";
		const KeyFrame& kf = map.kfs[selectedFrameIndex_];
		ImageRef sz = kf.image.pc().image_size();
		Vector<2> tl = kf.image.pc().retina_bounds().tl();
		Vector<2> br = kf.image.pc().retina_bounds().br();

		// Setup a projection matching that of the camera
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		double znear = 1e-2, zfar = 1e+2;
		glFrustum(tl[0]*znear, br[0]*znear, tl[1]*znear, br[1]*znear, znear, zfar);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glScalef(1,-1,-1);  // look down the positive Z axis, and put origin at top left

		RenderCamView();
	}





	void FloorPlanEditor::RenderFloorPlan(int viewType) {
		// Render the world
		if (viewType == ViewType::WIREFRAME) {
			glDisable(GL_DEPTH_TEST);
		} else {
			glEnable(GL_DEPTH_TEST);
		}

		glPointSize(1.0);
		glColorP(Colors::red());
		if (viewType == ViewType::WIREFRAME) {
			GL_PRIMITIVE(GL_POINTS) {
				BOOST_FOREACH(const Vector<3>& v, map.pts) {
					glVertexV(v);
				}
			}
		}

		glLineWidth(2.0);
		if (viewType == ViewType::WIREFRAME) {
			glColorP(Colors::aqua());
		}
		for (int i = 0; i+1 < floorplan->vertices_size(); i++) {
			Vector<2> a = asToon(floorplan->vertices(i));
			Vector<2> b = asToon(floorplan->vertices(i+1));
			if (!isnan(a[0]) && !isnan(b[0])) {
				if (viewType == ViewType::FILLED) {
					// TODO: chose the color based on the index of the vanishing point!
					glColorP(abs(a[0]-b[0])<abs(a[1]-b[1]) ? Colors::green() : Colors::red());
					GL_PRIMITIVE(GL_QUADS) {
						glVertex3f(a[0], a[1], floorplan->zfloor());
						glVertex3f(b[0], b[1], floorplan->zfloor());
						glVertex3f(b[0], b[1], floorplan->zceil());
						glVertex3f(a[0], a[1], floorplan->zceil());
					}
				} else {
					GL_PRIMITIVE(GL_LINES) {
						glVertex3f(a[0], a[1], floorplan->zfloor());
						glVertex3f(b[0], b[1], floorplan->zfloor());
						glVertex3f(b[0], b[1], floorplan->zceil());
						glVertex3f(a[0], a[1], floorplan->zceil());
					}
				}
			}
		}
		if (viewType == ViewType::WIREFRAME) {
			GL_PRIMITIVE(GL_LINES) {
				BOOST_FOREACH(const proto::Vec2& vert, floorplan->vertices()) {
					Vector<2> v = asToon(vert);
					if (!isnan(v[0])) {
						glVertex3f(v[0], v[1], floorplan->zfloor());
						glVertex3f(v[0], v[1], floorplan->zceil());
					}
				}
			}

			if (uiState == UIState::INSERT && mouseState == 2) {
				GL_PRIMITIVE(GL_LINES) {
					Vector<2> last = asToon(floorplan->vertices(floorplan->vertices_size()-1));
					glVertex3f(last[0], last[1], floorplan->zfloor());
					glVertex3f(endPt[0], endPt[1], floorplan->zfloor());
					glVertex3f(last[0], last[1], floorplan->zceil());
					glVertex3f(endPt[0], endPt[1], floorplan->zceil());
				}
			}
		}
	}

	void FloorPlanEditor::RenderFreeView() {
		RenderFloorPlan(ViewType::WIREFRAME);
	}

	void FloorPlanEditor::RenderCamView() {
		const KeyFrame& kf = map.kfs[selectedFrameIndex_];

		// Draw the frame
		if (camViewState == ViewType::WIREFRAME) {
			Vector<2> tl = kf.image.pc().retina_bounds().tl();
			Vector<2> br = kf.image.pc().retina_bounds().br();
			glColorP(Colors::white());
			camTextures.Select(&kf.image);
			WITHOUT(GL_BLEND) WITH(GL_TEXTURE_2D) GL_PRIMITIVE(GL_QUADS) {
				glTexCoord2f(0, 0);
				glVertex3f(tl[0], tl[1], 1);
				glTexCoord2f(0, 1);
				glVertex3f(tl[0], br[1], 1);
				glTexCoord2f(1, 1);
				glVertex3f(br[0], br[1], 1);
				glTexCoord2f(1, 0);
				glVertex3f(br[0], tl[1], 1);
			}
		}

		// Transform to camera coords
		Matrix<4> mv = Identity;
		mv.slice<0,0,3,3>() = kf.image.pc().pose().get_rotation().get_matrix();
		mv.slice<0,3,3,1>() = kf.image.pc().pose().get_translation().as_col();
		TransformGL(mv);

		// Draw the floorplan
		RenderFloorPlan(camViewState);
	}

	void FloorPlanEditor::RenderSideView() {
		glLineWidth(2.0);
		glColorP(Colors::aqua());
		GL_PRIMITIVE(GL_LINES) {
			glVertex3f(-10, 0, floorplan->zfloor());
			glVertex3f(10, 0, floorplan->zfloor());
		}
		glColorP(Colors::aqua());
		GL_PRIMITIVE(GL_LINES) {
			glVertex3f(-10, 0, floorplan->zceil());
			glVertex3f(10, 0, floorplan->zceil());
		}
	}

	void FloorPlanEditor::RenderPlanView() {
		glLineWidth(2.0);
		glPointSize(2.0);
		for (int i = 0; i+1 < floorplan->vertices_size(); i++) {
			Vector<2> a = asToon(floorplan->vertices(i));
			// Draw the line segment
			Vector<2> b = asToon(floorplan->vertices(i+1));
			if (!isnan(a[0]) && !isnan(b[0])) {
				if (i == selectedItem) {
					glColorP(Colors::fuchsia());
				} else if (i == hoverItem) {
					glColorP(Colors::white());
				} else {
					glColorP(Colors::aqua());
				}
				GL_PRIMITIVE(GL_LINES) {
					glVertexV(a);
					glVertexV(b);
				}
				GL_PRIMITIVE(GL_POINTS) {
					glVertexV(a);
					glVertexV(b);
				}
			}
		}

		if (uiState == UIState::INSERT && mouseState == 2) {
			glColorP(Colors::aqua());
			GL_PRIMITIVE(GL_LINES) {
				glVertexV(asToon(floorplan->vertices(floorplan->vertices_size()-1)));
				glVertexV(endPt);
			}
		}
	}




	void FloorPlanEditor::mapWidget_PreRender() {
		int i = selectedFrameIndex_;
		if (i >= 0 && i < mapWidget.kf_widgets.size()) {
			mapWidget.kf_widgets[i]->SetSelected(true);
		}
	}

	void FloorPlanEditor::anyView_KeyDown(byte c) {
		KeyDown.fire(c);
		if (keyMap_.find(c) != keyMap_.end()) {
			keyMap_[c].fire();
		}
	}

	void FloorPlanEditor::anyView_SpecialKeyDown(int key) {
		switch (key) {
		case GLUT_KEY_LEFT:
			SelectFrameByIndex((selectedFrameIndex_+map.kfs.size()-1)%map.kfs.size());
			break;
		case GLUT_KEY_RIGHT:
			SelectFrameByIndex((selectedFrameIndex_+1)%map.kfs.size());
			break;
		case GLUT_KEY_F2:
			enableSnap = !enableSnap;
		}
	}

	void FloorPlanEditor::SelectFrameByIndex(int index) {
		CHECK_GE(index, 0);
		CHECK_LT(index, map.kfs.size());
		CHECK_EQ(mapWidget.kf_widgets.size(), map.kfs.size());

		selectedFrameIndex_ = index;
		DLOG << "Selected frame ID " << map.kfs[index].id;
		Invalidate();
	}

	void FloorPlanEditor::Attach(proto::FloorPlan* fp) {
		// de-allocate the old floorplan (if there is one and we owned its memory)
		mem_floorplan.reset(NULL);
		// attach to the new floorplan
		floorplan = fp;
		ResetInternal();
		Invalidate();
	}

	void FloorPlanEditor::AttachAndManage(proto::FloorPlan* fp) {
		Attach(fp);
		mem_floorplan.reset(fp);  // MUST come after Attach()
	}
};
