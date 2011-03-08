#include <GL/gl.h>
#include <GL/glut.h>

#include <map>

#include "glut_window.h"
#include "vw_image_io.h"

#include "image_utils.tpp"
#include "image_transforms.tpp"

namespace indoor_context {
using namespace toon;
using namespace boost;

// Time between successive clicks to generate a DoubleClick event (in seconds)
lazyvar<float> gvDoubleClickTime("GlutWindow.DoubleClickTime");

// Static vars
map<int, GlutWindow*> GlutWindow::windows_;
scoped_ptr<thread> GlutWindow::glutLoopThread_;
InvokationSerializer GlutWindow::invoker_;
mutex GlutWindow::createWindowMutex_;
mutex GlutWindow::windowMapMutex_;
mutex GlutWindow::loopMutex_;
bool GlutWindow::glutLooping_ = false;

GlutWindow::GlutWindow(const string& title)
: handle_(-1),
  title_(title),
  displayMode_(GLUT_DOUBLE | GLUT_DEPTH),
  visible_(false),
  iconified_(false),
  size_(640,480),
  position_(0,0),
  cursor_(GLUT_CURSOR_LEFT_ARROW) {
	keyMap_[27].add(bind(&exit, 0));  // ESC quits the program
}

GlutWindow::~GlutWindow() {
	if (created()) {
		Destroy();
	}
}

void GlutWindow::Create() {
	if (!InGlutThread()) {
		// Note that this is okay because LoopInternal calls
		// invoker_.Process() _before_ calling glutMainLoop().
		DLOG << "Creating GLUT window (delegating)";
		RunInGlutThread(bind(&GlutWindow::Create, this));
		return;
	}

	DLOG << "Creating GLUT window for real";

	// Ensure GLUT is initialized
	Init();

	// Setup the initial window state
	glutInitDisplayMode(displayMode_);
	glutInitWindowSize(size_.x, size_.y);
	glutInitWindowPosition(position_.x, position_.y);
	handle_ = glutCreateWindow(title_.c_str());

	// Register the window in the map
	mutex::scoped_lock mapl(windowMapMutex_);
	CHECK(windows_.find(handle_) == windows_.end())
	<< "Window with ID=" << handle_ << " already present";
	windows_[handle_] = this;
	mapl.unlock();

	// Set some window parameters
	glutSetCursor(cursor_);
	glutSetWindowTitle(title_.c_str());

	// Register callbacks
	// (TODO: These only ever need to be called once I think -- move to Init?)
	glutDisplayFunc(&GlutWindow::HandleDisplay);
	glutReshapeFunc(&GlutWindow::HandleReshape);
	glutKeyboardFunc(&GlutWindow::HandleKeyboard);
	glutSpecialFunc(&GlutWindow::HandleSpecial);
	glutMouseFunc(&GlutWindow::HandleMouse);
	glutMotionFunc(&GlutWindow::HandleMotion);
	glutPassiveMotionFunc(&GlutWindow::HandlePassiveMotion);
	glutVisibilityFunc(&GlutWindow::HandleVisibility);
	glutEntryFunc(&GlutWindow::HandleEntry);
	glutIdleFunc(&GlutWindow::HandleIdle);
}

void GlutWindow::Run() {
	if (!created()) {
		Create();
	}
	Loop();
}

void GlutWindow::RunAsync() {
	if (!created()) {
		Create();
	}
	LoopAsync();
}

void GlutWindow::Destroy() {
	// We handle thread serialization slightly different here because
	// the window map must be updated immediately when Destroy is
	// called.
	int h = -1;

	CHECK(created());
	mutex::scoped_lock l(windowMapMutex_);
	windows_.erase(windows_.find(handle_));
	swap(h, handle_);
	l.unlock();

	if (InGlutThread()) {
		glutDestroyWindow(h);
	} else {
		RunInGlutThread(bind(&glutDestroyWindow, h));
	}
}

void GlutWindow::SetVisible(bool v) {
	if (!InGlutThread()) {
		RunInGlutThread(bind(&GlutWindow::SetVisible, this, v));
		return;
	}

	Select();
	visible_ = v;
	v ? glutShowWindow() : glutHideWindow();
}

void GlutWindow::Select() const {
	if (!InGlutThread()) {
		RunInGlutThread(bind(&GlutWindow::Select, this));
		return;
	}

	CHECK(created()) << "Select() called before window created";
	glutSetWindow(handle_);
}

void GlutWindow::Invalidate() const {
	if (!InGlutThread()) {
		RunInGlutThread(bind(&GlutWindow::Invalidate, this));
		return;
	}

	if (created()) {
		Select();
		glutPostRedisplay();
	}
}

void GlutWindow::SetSize(const ImageRef& size) {
	if (!InGlutThread()) {
		RunInGlutThread(bind(&GlutWindow::SetSize, this, size));
		return;
	}

	size_ = size;
	if (created()) {
		Select();
		glutReshapeWindow(size.x, size.y);
	}
}

void GlutWindow::SetPosition(const ImageRef& position) {
	if (!InGlutThread()) {
		RunInGlutThread(bind(&GlutWindow::SetPosition, this, position));
		return;
	}

	position_ = position;
	if (created()) {
		Select();
		glutPositionWindow(position.x, position.y);
	}
}

void GlutWindow::SetCursor(int cursor) {
	if (!InGlutThread()) {
		RunInGlutThread(bind(&GlutWindow::SetCursor, this, cursor));
		return;
	}

	cursor_ = cursor;
	if (created()) {
		Select();
		glutSetCursor(cursor);
	}
}

void GlutWindow::SetTitle(const string& title) {
	if (!InGlutThread()) {
		RunInGlutThread(bind(&GlutWindow::SetTitle, this, title));
		return;
	}

	title_ = title;
	if (created()) {
		Select();
		glutSetWindowTitle(title.c_str());
	}
}

void GlutWindow::SetDisplayMode(int v) {
	CHECK(!created()) << "Cannot set display mode after window created";
	displayMode_ = v;
}

void GlutWindow::Flush() const {
	if (!InGlutThread()) {
		RunInGlutThread(bind(&GlutWindow::Flush, this));
		return;
	}

	Select();
	glFlush();
}

void GlutWindow::CaptureFrameBuffer(ImageRGB<byte>& out) const {
	CHECK(InGlutThread()) << "Must capture the frame buffer from inside the GLUT thread.";
	Flush();
	ResizeImage(out, size_);
	glReadPixels(0,0, size_.x, size_.y, GL_BGRA, GL_UNSIGNED_BYTE, out.GetImageBuffer());
	ResetAlpha(out);  // the GL convention for alpha is different to ours
	FlipVertical(out);  // the GL convention for top and bottom is different to ours
}

void GlutWindow::OutputFrameBuffer(const string& filename) const {
	ImageRGB<byte> out;
	CaptureFrameBuffer(out);
	WriteImage(filename, out);
}

// private
void GlutWindow::NotifyDisplay() {
	Display.fire();
	// Swap buffers if double-buffering enabled
	if (displayMode_ & GLUT_DOUBLE) {
		glutSwapBuffers();
	} else {
		Flush();
	}
	AfterDisplay.fire();
}

// private
void GlutWindow::NotifyIdle() {
	Idle.fire();
}

// private
void GlutWindow::NotifyKeyboard(byte c, int x, int y) {
	KeyDown.fire(c);
	if (keyMap_.find(c) != keyMap_.end()) {
		keyMap_[c].fire();
	}
}

// private
void GlutWindow::NotifySpecial(int key, int x, int y) {
	SpecialKeyDown.fire(key);
	if (specialKeyMap_.find(key) != specialKeyMap_.end()) {
		specialKeyMap_[key].fire();
	}
}

// private
void GlutWindow::NotifyMouse(int button, int state, int x, int y) {
	mousePt_ = makeVector(x,y);
	raw.Mouse.fire(button, state, x, y);
	if (state == GLUT_DOWN) {
		mouseDownButton_ = button;
		mouseDragFlag_ = false;
		mouseDownPt_ = mousePt_;
	}

	if (state == GLUT_DOWN) {
		MouseDown.fire(button, mousePt_);
	} else if (state == GLUT_UP) {
		MouseUp.fire(button, mousePt_);
		if (!mouseDragFlag_) {
			// temporarily commented, need to re-implement VW::Timer
			/*if (mouseTimer_.GetAsSeconds() < *gvDoubleClickTime) {
				  DoubleClick.fire(button, mousePt_);
			} else {
			  mouseTimer_.Start();*/
				Click.fire(button, mousePt_);
				//}
		}
	}
}

// private
void GlutWindow::NotifyMotion(int x, int y) {
	mousePt_ = makeVector(x, y);
	mouseDragFlag_ = true;
	raw.Motion.fire(x, y);
	MouseDrag.fire(mouseDownButton_, mousePt_);
}

// private
void GlutWindow::NotifyPassiveMotion(int x, int y) {
	mousePt_ = makeVector(x, y);
	raw.PassiveMotion.fire(x, y);
	MouseMove.fire(mousePt_);
}

// private
void GlutWindow::NotifyReshape(int w, int h) {
	size_.x = w;
	size_.y = h;
	SizeChanged.fire();
}

// private
void GlutWindow::NotifyVisibility(int state) {
	visible_ = state == GLUT_VISIBLE;
	VisibilityChanged.fire();
}

// private
void GlutWindow::NotifyEntry(int state) {
	if (state == GLUT_ENTERED) {
		MouseIn.fire();
	} else {
		MouseOut.fire();
	}
}

// static
void GlutWindow::Init() {
	static int argc = 0;
	static char* argv = NULL;
	Init(&argc, &argv);
}

// static
void GlutWindow::Init(int *argc, char **argv) {
	static bool v = false;
	if (!v) {
		glutInit(argc, argv);
		v = true;
	}
}

// static
void GlutWindow::Loop() {
	mutex::scoped_lock l(loopMutex_);
	if (glutLooping_) {
		// Ugly hack to create a deadlock and go to sleep forever
		mutex deadlock;
		deadlock.lock();
		deadlock.lock();
	} else {
		glutLooping_ = true;
	}
	l.unlock();  // Without this the mutex will never be unlocked

	LoopInternal();
}

// static
void GlutWindow::LoopOrDie() {
	mutex::scoped_lock l(loopMutex_);
	CHECK(!glutLooping_) << "The GLUT loop is already running";
	glutLooping_ = true;
	l.unlock();  // Without this the mutex will never be unlocked
	LoopInternal();
}

// static
void GlutWindow::LoopAsync() {
	mutex::scoped_lock l(loopMutex_);
	if(!glutLooping_) {
		glutLooping_ = true;
		glutLoopThread_.reset(new thread(&GlutWindow::LoopInternal));
	}
}

// static
void GlutWindow::LoopAsyncOrDie() {
	mutex::scoped_lock l(loopMutex_);
	CHECK(!glutLooping_) << "The GLUT loop is already running";
	glutLooping_ = true;
	glutLoopThread_.reset(new thread(&GlutWindow::LoopInternal));
}

// static
void GlutWindow::LoopInternal() {
	// Initialize GLUT if not already done
	Init();
	// Should never be called more than once
	invoker_.RegisterHomeThread(this_thread::get_id());
	// Clear out any pending calls since we're now in the main loop
	// (this is _important_ because there must be a window created
	// before calling glutMainLoop!)
	invoker_.Process();
	// Enter the main loop
	glutMainLoop();
}

// static
void GlutWindow::RunInGlutThread(boost::function<void()> f) {
	invoker_.InvokeLater(f);
}

// static
bool GlutWindow::InGlutThread() {
	return invoker_.InHomeThread();
}

// static
void GlutWindow::HandleDisplay() {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) CurrentWindow()->NotifyDisplay();
}
// static
void GlutWindow::HandleIdle() {
	// Deal with GLUT functions called from other threads
	invoker_.Process();
	// Allow the current window to do background tasks
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifyIdle();
}
// static
void GlutWindow::HandleKeyboard(byte c, int x, int y) {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifyKeyboard(c,x,y);
}
// static
void GlutWindow::HandleSpecial(int key, int x, int y) {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifySpecial(key,x,y);
}
// static
void GlutWindow::HandleMouse(int b, int s, int x, int y) {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifyMouse(b,s,x,y);
}
// static
void GlutWindow::HandleMotion(int x, int y) {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifyMotion(x,y);
}
// static
void GlutWindow::HandlePassiveMotion(int x, int y) {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifyPassiveMotion(x,y);
}
// static
void GlutWindow::HandleReshape(int nx, int ny) {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifyReshape(nx, ny);
}
// static
void GlutWindow::HandleVisibility(int state) {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifyVisibility(state);
}
// static
void GlutWindow::HandleEntry(int state) {
	GlutWindow* w = CurrentWindow();
	if (w != NULL) w->NotifyEntry(state);
}

// static, private
GlutWindow* GlutWindow::CurrentWindow() {
	// This member is private because it makes a call to
	// glutGetWindow, which must be made from within the GLUT
	// thread. This can be overcome either by implementing a mechanism
	// to call glutGetWindow from within the GLUT thread and obtain
	// its result in the original thread, or just by calling
	// CurrentWindow from within the GLUT thread.
	int cur_id = glutGetWindow();
	if (cur_id == 0) {
		return NULL;
	} else {
		mutex::scoped_lock l(windowMapMutex_);
		map<int,GlutWindow*>::iterator it = windows_.find(cur_id);
		if (it == windows_.end()) {
			// This can happen if the user calls glutCreateWindow manually
			return NULL;
		} else {
			return it->second;
		}
	}
}
}
