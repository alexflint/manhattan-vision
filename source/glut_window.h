#pragma once

#include <boost/thread.hpp>

#include <GL/glut.h>

#include "common_types.h"
#include "invokation_serializer.h"

#include "vw_image.tpp"
#include "event.tpp"

namespace indoor_context {
	// Manages all aspects of a GLUT window.
	//   GlutWindow myWindow("The title");
	//   myWindow.Display.add(bind(&myDrawFunc, foo, bar));
	//   myWindow.Run();   // begins GLUT loop
	//
	// Safe to create window from GLUT callbacks, and create multiple
	// windows before calling run(). Do not run() windows from multiple
	// different threads. One GLUT thread handles all windows.
	class GlutWindow {
	public:
		// Create a GLUT window with the given title.
		GlutWindow(const string& title="GLUT Window");
		// Destroys the window
		~GlutWindow();

		// Call glutInit() with empty arguments
		static void Init();
		// Call glutInit() with arguments
		static void Init(int *argc, char **argv);
		 // Enter the GLUT loop in this thread, or, if it is already
		 // running in a different thread, just sleep forever.
		static void Loop();
		 // Enter the GLUT loop in this thread or die if it is already started.
		static void LoopOrDie();
		 // If the GLUT loop is not already started then start it in a
		 // parallel thread
		static void LoopAsync();
		// Start the GLUT loop in a different thread, or die if it is already started.
		static void LoopAsyncOrDie();
		// Returns true when called from the GLUT event loop, false otherwise.
		static bool InGlutThread();
		// Run a function in the GLUT loop next time glutIdleFunc is
		// called.
		static void RunInGlutThread(boost::function<void()> f);

		// Create the window. Might not actually show the window if the
		// GLUT loop is not running.
		void Create();
		// Create the window and enter the GLUT loop
		void Run();
		// Create the window and, if the GLUT loop is not already running,
		// start it in a parallel thread.
		void RunAsync();
		// Destroy the window
		void Destroy();
		// Return true if this window has a valid glut handle
		inline bool created() const { return handle_ != -1; }

		// Show/hide the window
		inline bool visible() const { return visible_; }
		inline bool iconified() const { return iconified_; }
		void Show() { SetVisible(true); }
		void Hide() { SetVisible(false); }
		void SetVisible(bool v);

		// Select this window as the target of future glut commands. Note
		// that this has no effect on which X window is focussed.
		void Select() const;
		// Request the window to be redrawn
		void Invalidate() const;

		// Get the Glut ID of this window
		inline int id() const { return handle_; }

		// Get the mouse location
		inline const Vec2& mouse_location() const { return mousePt_; }

		// Get/set window size
		inline const ImageRef& size() const { return size_; }
		void SetSize(const ImageRef& size);

		// Get/set window position
		inline const ImageRef& position() const { return position_; }
		void SetPosition(const ImageRef& position);

		// Get/set cursor
		inline int cursor() const { return cursor_; }
		void SetCursor(int cursor);

		// Get/set title
		inline const string& title() const { return title_; }
		void SetTitle(const string& title);

		// Get/set display mode as passed to glutInitDisplayMode
		// SetDisplayMode() must only be called before Create()
		inline const int display_mode() const { return displayMode_; }
		void SetDisplayMode(int v);

		// Flush any cached GL commands
		void Flush() const;

		// Bind a function to a key
		inline Event<>& KeyStroke(int key) { return keyMap_[key]; }
		// Bind a function to a special key (arrows, function keys, etc)
		inline Event<>& SpecialKeyStroke(int key) { return specialKeyMap_[key]; }

		// Copy the frame buffer to an image. The image will be resized
		// according to the current window size.
		void CaptureFrameBuffer(ImageRGB<byte>& out) const;

		// Write the frame buffer to a file
		void OutputFrameBuffer(const string& filename) const;

		// Events
		Event<> Display;  // fired when the window needs to be redrawn.
		Event<> AfterDisplay;  // fired after all gl* calls complete and flushed
		Event<> Idle;
		Event<> MouseIn;
		Event<> MouseOut;
		Event<void(byte c)> KeyDown;
		Event<void(int key)> SpecialKeyDown;  // arrows, function keys, etc
		Event<void(Vec2 pos)> MouseMove;
		Event<void(int button, Vec2 pos)> MouseDown;
		Event<void(int button, Vec2 pos)> MouseUp;
		Event<void(int button, Vec2 pos)> MouseDrag;
		Event<void(int button, Vec2 pos)> Click;
		Event<void(int button, Vec2 pos)> DoubleClick;
		Event<> VisibilityChanged;
		Event<> SizeChanged;
		Event<> Closing;

		// Raw events
		struct {
			Event<void(int button, int state, int x, int y)> Mouse;
			Event<void(int x, int y)> Motion;
			Event<void(int x, int y)> PassiveMotion;
		} raw;
	private:
		 // Enter the GLUT loop in this thread (will never return)
		static void LoopInternal();

		// GLUT handlers (these delegate to the Notify* methods for the active window)
		static void HandleDisplay();
		static void HandleIdle();
		static void HandleKeyboard(byte c, int x, int y);
		static void HandleSpecial(int key, int x, int y);  // arrows, function keys, etc
		static void HandleMouse(int button, int state, int x, int y);
		static void HandleMotion(int x, int y);
		static void HandlePassiveMotion(int x, int y);
		static void HandleReshape(int w, int h);
		static void HandleVisibility(int state);
		static void HandleEntry(int state);

		// Get the current active window, or NULL if either no window
		// exists or the previously created window was destroyed.
		static GlutWindow* CurrentWindow();
		// Return true if there is a current window (false only when
		// either no window exists or the previously created window was
		// destroyed).
		static bool HasCurrentWindow();

		// Handlers for GLUT events. These update internal state and fire
		// the relevant events.
		void NotifyDisplay();
		void NotifyIdle();
		void NotifyKeyboard(byte c, int x, int y);
		void NotifySpecial(int key, int x, int y);  // arrows, function keys, etc
		void NotifyMouse(int button, int state, int x, int y);
		void NotifyMotion(int x, int y);
		void NotifyPassiveMotion(int x, int y);
		void NotifyReshape(int w, int h);
		void NotifyVisibility(int state);
		void NotifyEntry(int state);

		// All windows indexed by ID
		static map<int, GlutWindow*> windows_;
		// The invokation manager for multi-threading
		static InvokationSerializer invoker_;
		// The global mutex for access to windows_
		static boost::mutex windowMapMutex_;
		// The global mutex for window creation.
		static boost::mutex createWindowMutex_;
		// The global mutex for entry to Loop(), LoopAsync(), etc. This is
		// locked at most once and never unlocked.
		static boost::mutex loopMutex_;
		// The thread in which GLUT is running, if that thread was created
		// by GLUT window, or NULL otherwise.
		static scoped_ptr<boost::thread> glutLoopThread_;
		// True if Loop() has been called. Note that this might be true
		// while glutLoopThread_ might be NULL if the user called Loop()
		// directly.
		static bool glutLooping_;

		// GLUT window ID
		int handle_;
		// Window title
		string title_;
		// Display mode as passed to glutInitDisplayMode
		int displayMode_;
		// Window visibility
		bool visible_;
		// Is window iconified?
		bool iconified_;
		// Current window size
		ImageRef size_;
		// Current window position
		ImageRef position_;
		// Current GLUT cursor
		int cursor_;

		// Last position at which the mouse went down
		Vec2 mouseDownPt_;
		// Last mouse button pressed
		int mouseDownButton_;
		// Whether the mouse has been dragged since the last mouseDown event
		bool mouseDragFlag_;
		// Current position of the mouse
		Vec2 mousePt_;
		// Time of the last click, used for detecting double-clicks
		//VW::Timer mouseTimer_;

		// Map from keys to handlers
		map<int, Event<> > keyMap_;
		// Map from keys to handlers for arrows, function keys, etc
		map<int, Event<> > specialKeyMap_;
	};
}
