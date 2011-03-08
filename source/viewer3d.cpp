#include <GL/glut.h>

#include <boost/bind.hpp>

#include <LU.h>

#include "viewer3d.h"
#include "common_types.h"
#include "geom_utils.h"
#include "widget3d.h"

#include "gl_utils.tpp"
//#include "numeric_utils.tpp"

namespace indoor_context {
using namespace TooN;

double kRotateSpeed = 0.4;
double kPanSpeed = 0.1;




///////////////// Viewer3D ///////////////////////
Viewer3D::Viewer3D()
: viewCentre(Zeros),
  viewRotation(Zeros),
  viewLogDist(300),
  viewOrtho(false),
  configured(false),
  window_(new GlutWindow("3D Viewer")),
  selection_(NULL),
  hoverWidget_(NULL),
  bgcolor_(0,0,0),
  navigable_(true) {
	// The root widget is a special case when it comes to getting
	// attached to the viewer since it propogates its pointer to all
	// other widgets.
	rootWidget_->NotifyAttached(this);
}

Viewer3D::Viewer3D(const string& title)
: viewCentre(Zeros),
  viewRotation(Zeros),
  viewLogDist(300),
  viewOrtho(false),
  configured(false),
  window_(new GlutWindow("3D Viewer")),
  selection_(NULL),
  hoverWidget_(NULL),
  bgcolor_(0,0,0),
  navigable_(true) {
	window_->SetTitle(title);
	// The root widget is attached as a special case since it propogates its pointer to all
	// other widgets.
	rootWidget_->NotifyAttached(this);
}

Viewer3D::~Viewer3D() {
}

// static
void Viewer3D::Init(int *argc, char **argv) {
	GlutWindow::Init(argc, argv);
}

Vector<2> Viewer3D::ProjectToScreen(const Vector<3>& v) const {
	projector_->Configure(window().size());
	return projector_->Project(v);
}

Vector<2> Viewer3D::ProjectToScreen(const Vector<4>& v) const {
	return ProjectToScreen(toon::project(v));
}

Vector<2> Viewer3D::WindowToViewport(const Vector<2>& w) const {
	Vector<4> vp = GetGLViewport();
	return makeVector(2.0*(w[0]-vp[0])/vp[2] - 1.0,
			2.0*(w[1]-vp[1])/vp[3] - 1.0);
}

// Project a mouse location in window coordinates to a plane in
// 3D. Mouse coords should be as passed to glutMouseMotion
// etc. Returns a 3D point on the plane specified by planeEqn.
Vector<3> Viewer3D::MouseToPlane(const Vector<2> mousePt,
                                 const Vector<4>& planeEqn) {
	Vector<2> viewPt = makeVector(mousePt[0], window().size().y-mousePt[1]); // invert Y coordinate
	Vector<2> eyePt = WindowToViewport(viewPt);
	Matrix<4> A = GetGLProjection() * GetGLModelView();
	A.slice<2,0,1,4>() = planeEqn.as_row();
	Vector<4> worldPt = LU<>(A).backsub(makeVector(eyePt[0], eyePt[1], 0, 1));
	return project(worldPt);
}

void Viewer3D::SetNavigable(bool v) {
	navigable_ = v;
}

void Viewer3D::Create() {
	window().Display.add(bind(&Viewer3D::Window_Display, this));
	window().MouseMove.add(bind(&Viewer3D::Window_MouseMove, this, _1));
	window().MouseDown.add(bind(&Viewer3D::Window_MouseDown, this, _1, _2));
	window().MouseUp.add(bind(&Viewer3D::Window_MouseUp, this, _1, _2));
	window().MouseDrag.add(bind(&Viewer3D::Window_MouseDrag, this, _1, _2));
	window().Click.add(bind(&Viewer3D::Window_Click, this, _1, _2));
	window().DoubleClick.add(bind(&Viewer3D::Window_DoubleClick, this, _1, _2));
	window().SizeChanged.add(bind(&Viewer3D::Window_SizeChanged, this));
	window().Create();

	configured = true;
}

void Viewer3D::Run() {
	if (!configured) {
		Create();
	}
	window().Run();
}

void Viewer3D::RunAsync() {
	if (!configured) {
		Create();
	}
	window().RunAsync();
}

// static
void Viewer3D::RunWidget(Widget3D& w) {
	Viewer3D v;
	v.Add(w);
	v.Run();
}

void Viewer3D::Invalidate() {
	if (configured) {
		window().Invalidate();
	}
}


void Viewer3D::Select(Widget3D& w) {
	if (&w != selection_) {
		if (selection_ != NULL) {
			selection_->NotifySelectedChanged(false);
		}
		w.NotifySelectedChanged(true);
		selection_ = &w;
		Invalidate();
	}
}

void Viewer3D::Deselect() {
	if (selection_ != NULL) {
		selection_->NotifySelectedChanged(false);
		Invalidate();
	}
	selection_ = NULL;
}

void Viewer3D::SetBgColor(const PixelRGB<byte>& color) {
	glClearColor(color.r/255.0, color.g/255.0, color.b/255.0, 0.0);
	bgcolor_ = color;
	Invalidate();
}

Widget3D& Viewer3D::Add(Widget3D& child, char toggleKey) {
	return rootWidget_->Add(child, toggleKey);
}

Widget3D& Viewer3D::AddOwned(Widget3D* child, char toggleKey) {
	return rootWidget_->AddOwned(child, toggleKey);
}

Widget3D& Viewer3D::Add(boost::function<void()> renderFunc, char toggleKey) {
	return rootWidget_->Add(renderFunc, toggleKey);
}

const vector<Widget3D*>& Viewer3D::children() const {
	return rootWidget_->children();
}

Widget3D* Viewer3D::GetWidgetAt(const toon::Vector<2>& mouse) {
	return rootWidget_->GetWidgetAt(mouse);
}

void Viewer3D::ConfigureProjection() {
	glViewport(0, 0, window().size().x, window().size().y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double aspect = 1.0*window().size().x / window().size().y;
	if (viewOrtho) {
		glOrtho(-10.0*aspect, 10.0*aspect, -10.0, 10.0, 1e-2, 100.0);
	} else {
		gluPerspective(45.0, aspect, 1e-2, 100.0);
	}
	glError();
	glMatrixMode(GL_MODELVIEW);
	Invalidate();
}



void Viewer3D::Window_Display() {
	// Configure GL
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glError();

	// Setup the camera viewpoint
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	double viewZ = exp(viewLogDist/100.0);
	gluLookAt(0,0,-viewZ,   0,0,0,   0,1,0);
	glRotatef(viewRotation[0], 0, 1, 0);
	glRotatef(viewRotation[1], 1, 0, 0);
	glRotatef(viewRotation[2], 0, 0, 1);
	if (viewOrtho) {
		double s = exp(3.0)/viewZ;
		glScaled(s, s, s);
	}
	glTranslated(viewCentre[0], viewCentre[1], viewCentre[2]);
	glError();

	// Antialiasing does not play nicely with depth testing in
	// OpenGL. Drawing an antialiased line, for example, will write to
	// the depth buffer for each pixel rendered, including pixels the
	// antialiasing that are nearly transparent. Other primitives will
	// then be occluded by these near-transparent pixels, causing
	// apparent gaps at the edge of lines. Similar things happen for
	// antialiased polygons and points. The solution we use here is to
	// draw everything twice: first without antialiasing and with
	// depth buffer writing turned on, and then again with
	// antialiasing but with depth buffer writing turned off. (Note
	// that though we are not _writing_ to the depth buffer, we are
	// still doing depth _testing_ in both cases).

	// Draw the non-antialiased stuff
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_POINT_SMOOTH);
	glDisable(GL_POLYGON_SMOOTH);
	glDepthMask(GL_TRUE);

	isAntialiasing = false;
	rootWidget_->NotifyPreRender();
	rootWidget_->NotifyRender();

	// Draw the antialiased stuff
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glDepthMask(GL_FALSE);

	isAntialiasing = true;
	rootWidget_->NotifyPreRender();
	rootWidget_->NotifyRender();
	isAntialiasing = false;

	glDepthMask(GL_TRUE);
}

void Viewer3D::Window_MouseDown(int button, Vector<2> mousePt) {
	Widget3D* hit = GetWidgetAt(mousePt);
	if (hit) {
		hit->NotifyMouseDown(button, mousePt);
	}
	prevMousePt_ = mousePt;
	Invalidate();
}

void Viewer3D::Window_MouseUp(int button, Vector<2> mousePt) {
	Widget3D* hit = GetWidgetAt(mousePt);
	if (hit) {
		hit->NotifyMouseUp(button, mousePt);
	}
	Invalidate();
}

void Viewer3D::Window_Click(int button, Vector<2> mousePt) {
	Widget3D* hit = GetWidgetAt(mousePt);
	if (hit) {
		hit->NotifyClick(button, mousePt);
	}
	Invalidate();
}

void Viewer3D::Window_DoubleClick(int button, Vector<2> mousePt) {
	Widget3D* hit = GetWidgetAt(mousePt);
	if (hit) {
		hit->NotifyDoubleClick(button, mousePt);
	}
	Invalidate();
}

void Viewer3D::Window_MouseDrag(int button, Vector<2> mousePt) {
	// now fire the event for the specific widget
	if (hoverWidget_ != NULL && hoverWidget_->draggable()) {
		hoverWidget_->NotifyMouseDrag(button, mousePt);
	} else if (navigable_) {
		if (button == GLUT_RIGHT_BUTTON) {
			viewLogDist += mousePt[1] - prevMousePt_[1];
		} else if (button == GLUT_LEFT_BUTTON) {
			if (viewOrtho) {
				viewCentre.slice<0,2>() += kPanSpeed * (mousePt - prevMousePt_);
			} else {
				viewRotation[0] += kRotateSpeed*(mousePt[0] - prevMousePt_[0]);
				viewRotation[1] += kRotateSpeed*(mousePt[1] - prevMousePt_[1]);
			}
		}
	}
	prevMousePt_ = mousePt;
	Invalidate();
}

void Viewer3D::Window_MouseMove(Vector<2> mousePt) {
	// TODO: should only fire the event for the selected widget
	rootWidget_->Traverse(bind(&Widget3D::NotifyMouseMove,
			_1, ref(mousePt)));

	bool redraw = false;
	Widget3D* w = GetWidgetAt(mousePt);
	if (hoverWidget_ != NULL && hoverWidget_ != w) {
		hoverWidget_->NotifyHoverChanged(false);
		redraw = true;
	}
	if (w != NULL && w != hoverWidget_) {
		w->NotifyHoverChanged(true);
		hoverWidget_ = w;
		redraw = true;
	}
	hoverWidget_ = w;
	if (redraw) {
		Invalidate();
	}
}

void Viewer3D::Window_SizeChanged() {
	ConfigureProjection();
	Invalidate();
}







GluProjector::GluProjector() {
}

GluProjector::GluProjector(const ImageRef& winSize) {
	Configure(winSize);
}

void GluProjector::Configure(const ImageRef& winSize) {
	windowSize = winSize;
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
}

Vector<2> GluProjector::Project(const Vector<3>& v) const {
	Vector<3> p;
	gluProject(v[0],v[1],v[2], modelview, projection, viewport, &p[0],&p[1],&p[2]);
	p[1] = windowSize.y-p[1];  // Move (0,0) to top-left rather than bottom-left
	glError();

	// Note that the third parameter of p is a depth parameter, NOT a
	// homogeneous third coord. We ignore it in this case.
	return p.slice<0,2>();
}




void TextureManager::Select(const ImageBundle* image) {
	GLuint texId = LoadOrLookup(image);
	glBindTexture(GL_TEXTURE_2D, texId);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glError();
}

GLuint TextureManager::LoadOrLookup(const ImageBundle* image) {
	if (textureIds_.find(image) == textureIds_.end()) {
		textureIds_[image] = image->LoadGLTexture();
	}
	return textureIds_[image];
}

GLuint TextureManager::Reload(const ImageBundle* image) {
	GLuint texId = -1;
	if (textureIds_.find(image) != textureIds_.end()) {
		texId = textureIds_[image];
	}
	textureIds_[image] = image->LoadGLTexture(texId);
}

void TextureManager::Delete(const ImageBundle* image) {
	map<const ImageBundle*,unsigned>::iterator it = textureIds_.find(image);
	if (it != textureIds_.end()) {
		GLuint texId = textureIds_[image];
		glDeleteTextures(1, &texId);
		textureIds_.erase(it);
	}
}

void TextureManager::Render(const ImageBundle* image,
                            const GlutWindow* window) {
	const float kDepth = 1.0;

	// Select the window
	window->Select();

	// Load the texture
	Select(image);

	glShadeModel(GL_FLAT);
	WITHOUT(GL_DEPTH_TEST) WITHOUT(GL_COLOR_MATERIAL) WITH(GL_TEXTURE_2D) {
		// Render the textured quad
		glBegin(GL_QUADS);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-1,  -1, kDepth);
		glTexCoord2f(0.0,1.0);
		glVertex3f(-1, 1, kDepth);
		glTexCoord2f(1.0,1.0);
		glVertex3f(1,1, kDepth);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(1, -1, kDepth);
		glEnd();
	}
	glShadeModel(GL_SMOOTH);
}

void TextureManager::RenderFullScreen(const ImageBundle* image,
                                      const GlutWindow* window) {
	GL_MATRIX_SCOPE {
		glLoadIdentity();
		glRotatef(180.0, 0.0, 0.0, 1.0);
		Render(image, window);
	}
}
}
