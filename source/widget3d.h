#pragma once

#include <map>

#include <boost/function.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "polygon.tpp"
#include "event.tpp"

namespace indoor_context {

class Viewer3D;
class GluProjector;

// Interface for widgets
class Widget3D {
public:
	// Events
	Event<> PreRender; // just before render, don't actually draw stuff here
	Event<> Render;  // make GL calls here
	Event<> MouseDown;
	Event<> MouseUp;
	Event<> MouseMove;
	Event<> MouseDrag;
	Event<> Click;
	Event<> DoubleClick;
	Event<> KeyStroke;  // not yet implemented
	Event<> SelectedChanged;
	Event<> HoverChanged;
	Event<> VisibilityChanged;
	Event<> Attached;
	Event<> Deattached;

	// Constructor
	Widget3D();
	virtual ~Widget3D() { }

	// Invalidate this widget so it will be redrawn. Actually just
	// invalidates the entire display.
	void Invalidate();

	// Apply a function to this widget and all children in depth-first order.
	void Traverse(const function<void(Widget3D*)>& f);

	// Show this widget. The OnRender() method will be called during rendering.
	void Show() { SetVisible(true); };
	// Hide this widget. The OnRender() method will not be called during rendering.
	void Hide() { SetVisible(false); }
	// Toggle visibility of this widget
	void ToggleVisible() { SetVisible(!_visible); }
	// Set visibility
	void SetVisible(bool v);
	// Whether this object is visible
	bool visible() const { return _visible; }

	// Whether the mouse is hovering over this widget
	bool hovering() const { return _hovering; }

	// Set this object as selected.
	void SetSelected(bool v);
	// Whether this object is selected
	bool selected() const { return _selected; }

	// Set whether this object can be selected by the user
	void SetSelectable(bool v) { _selectable = v; Invalidate(); }
	// Whether this object can be selected by the user
	bool selectable() const { return _selectable; }

	// Set whether this object can be dragged (determines whether it
	// recieves MouseDrag events)
	void SetDraggable(bool v) { _draggable = v; Invalidate(); }
	// Whether this object can be dragged
	bool draggable() const { return _draggable; }

	// The viewer at the root of the hierarchy. Will be a ref to NULL
	// iff attached() returns false.
	Viewer3D& viewer() const { return *_viewer; }
	// Determine whether this widget is attached to a viewer
	bool attached() const { return _viewer != NULL; }

	// Add a widget. The optional second parameter is a key that will
	// toggle the widget's visibility.
	Widget3D& Add(Widget3D& child, char toggleKey=0);
	// Add a widget, and delete it when this object is destroyed. The
	// optional second parameter is a key that will toggle the
	// widget's visibility.
	Widget3D& AddOwned(Widget3D* child, char toggleKey=0);
	// Add a function as a widget. This function will be called on
	// each redraw. It gets treated like a widget that can't recieve
	// focus or do any of the other fancy things that a full widget
	// implementation can do. You may use boost:bind to have
	// parameters passed to this method:
	//
	// void DrawMyFancyGraphics(Foo myFoo, const string& bar, int baz) { ... }
	//
	// int main(...) {
	//   Viewer3D myVizualization;
	//   myVizualization.Add(boost::bind(&DrawMyFancyGraphics, theFoo, "some string", 12345));
	// }
	//
	// Returns a widget that wraps this function
	Widget3D& Add(boost::function<void()> renderFunc, char toggleKey=0);
	// Remove the child at offset i
	void RemoveChild(int i);
	// Remove all children
	void ClearChildren();
	// List of child widgets
	const vector<Widget3D*>& children() const { return _children; }

	// At present z-order only controls click handling, not
	// display. Objects with lower z-order recieve clicks first
	int zorder() { return _zorder; }
	void SetZorder(int zo) { _zorder = zo; Invalidate(); }

	// Return true if this mouse location represents a hit. Use
	// Viewer3D::ProjectToScreen() during OnRender() to implement this.
	virtual bool HitTest(const toon::Vector<2>& mouse) const;
	// By invoking HitTest(mouse), determine which widget is at the
	// given mouse location, or return NULL if there isn't one.
	Widget3D* GetWidgetAt(const toon::Vector<2>& mouse);
	// TODO: add a const version of the above, which also returns "const Widget3D*"

	// Render the widget (all GL calls go in here)
	virtual void OnPreRender() { }
	// Render the widget (all GL calls go in here)
	virtual void OnRender() { }
	// Called when the object is clicked (as determined by HitTest)
	virtual void OnClick(int button, const toon::Vector<2>& mouse) { }
	// Called when the object is double-clicked
	virtual void OnDoubleClick(int button, const toon::Vector<2>& mouse) { }

	// Fire various events and change some internal state. These are
	// usually called only from Viewer3D. Some of these are recursive.
	void NotifyRender();
	void NotifyPreRender();
	void NotifyMouseDown(int button, const toon::Vector<2>& mouse);
	void NotifyMouseUp(int button, const toon::Vector<2>& mouse);
	void NotifyMouseMove(const toon::Vector<2>& mouse);
	void NotifyMouseDrag(int button, const toon::Vector<2>& mouse);
	void NotifyClick(int button, const toon::Vector<2>& mouse);
	void NotifyDoubleClick(int button, const toon::Vector<2>& mouse);
	void NotifySelectedChanged(bool v);  // (de)selected by the user
	void NotifyHoverChanged(bool v);  // mouse is (de)hovering
	void NotifyAttached(Viewer3D* viewer);  // attached to a viewer
	void NotifyDeattached();  // deattached from any viewer
private:
	// See accessors for descriptions
	Viewer3D* _viewer;
	bool _hovering;
	bool _selectable;
	bool _selected;
	bool _visible;
	bool _draggable;
	int _zorder;
	vector<Widget3D*> _children;

	// This vector contains the subset of _children owned by this
	// widget. These will be deleted when this widget is
	// deleted. Should be no reason to read from this vector, it's
	// just a memory manager.
	ptr_vector<Widget3D> _owned_children;
};




// Represents a quad
class QuadWidget : public Widget3D {
public:
	toon::Vector<3> a, b, c, d;
	PixelRGB<byte> color;
	float border_width;

	QuadWidget();
	QuadWidget(const toon::Vector<3>& a,
			const toon::Vector<3>& b,
			const toon::Vector<3>& c,
			const toon::Vector<3>& d);
	QuadWidget(const toon::Matrix<3,2>& m,
			float left,
			float right,
			float top,
			float bottom);

	void OnRender();
};


// Represents a widget displayed as a dot
class PointWidget : public Widget3D {
public:
	toon::Vector<3> p;
	float size;
	PixelRGB<byte> color;
	int depth_offset;
	PointWidget(const toon::Vector<3>& x,
			float sz,
			const PixelRGB<byte>& c,
			int doffs=0)
	: p(x), size(sz), color(c), depth_offset(doffs) {
	}
	void OnRender();
};


// This widget represents a set of similar line segments. It allows
// many GL operations to be aggregated over the set of line
// segments.
class LineWidget;
class LineWidgetBloc : public Widget3D {
public:
	float width;
	int depth_offset;  // passed to glPolygonOffset
	vector<LineWidget*> members;

	LineWidgetBloc(float w=1.0, int doffs=0)
	: width(w), depth_offset(doffs) { }

	LineWidget& AddNewMember();
	LineWidget& AddNewMember(const toon::Vector<3>& start,
			const toon::Vector<3>& end,
			const PixelRGB<byte>& color);

	virtual void OnRender();
};


// A generic line widget
class LineWidget : public Widget3D {
public:
	static const float kHitThresh = 2.5;

	LineSeg lineseg;
	float width;
	PixelRGB<byte> color;
	PixelRGB<byte> selectColor;
	int depthOffset;  // passed to glPolygonOffset
	LineWidgetBloc* bloc;

	// Current projection of world points
	LineSeg screen_line;

	// Updated during PreRender
	bool subsumed;

	LineWidget();

	LineWidget(const toon::Vector<3>& start,
	           const toon::Vector<3>& end,
	           float width,
	           const PixelRGB<byte>& color);

	LineWidget(const LineSeg& lineseg,
	           float width,
	           const PixelRGB<byte>& color);

	bool HitTest(const toon::Vector<2>& mouse) const;
	void OnPreRender();
	void OnRender();
};

// The ground plane widget.
class GroundPlaneWidget : public Widget3D {
public:
	virtual void OnRender();
};

}
