#pragma once

#include "common_types.h"
#include "viewer3d.h"
#include "widget3d.h"
#include "map.h"

namespace indoor_context {
// Represents a clickable keyframe quad
class KeyFrameWidget : public Widget3D {
public:
	// Projection of quad boundary onto the screen
	Vec2 pa, pb, pc, pd;
	// Distance from camera centre to image plane, in GL coordinates
	double retina_z;
	// The keyframe we represent
	const KeyFrame& kf;
	// The child line widgets
	vector<LineWidget*> line_widgets;
	// The child guided line widgets
	vector<LineWidget*> guided_line_widgets;
	// The aggregator for line widgets from kf.line_detector
	LineWidgetBloc line_bloc;
	// The aggregator for line widgets from kf.guided_line_detector
	LineWidgetBloc guided_line_bloc;

	// Border parameters
	float border_width;
	PixelRGB<byte> border_color;

	// Whether to show line segments in this keyframe:
	// 0=no lines, 1=kf.line_detector, 2=kf.guided_line_detector
	int line_state;
	// Whether to render the texton map rather than the image
	bool show_textons;

	// Initialize the keyframe widget
	// retina_z: the distance from the camera center at which the
	// image is rendered (aesthetic only).
	KeyFrameWidget(const KeyFrame& kf, double retina_z = 1.0);
	// Draw the keyframe as a quad in 3D
	virtual void OnRender();
	// Return true if ths mouse location is inside the rendered keyframe
	virtual bool HitTest(const Vec2& mouse) const;
	// Click handler: prints the image filename
	virtual void OnClick(int button, const Vec2& mouse);
	// Double-click handler: makes this keyframe the center of rotation
	virtual void OnDoubleClick(int button, const Vec2& mouse);

	// Click handler for lines in the image
	void Line_Click(int index, const string& label);

	// Transform to a coordinate frame in which this keyframe's camera
	// centre is at the origin and the image plane is at z=1.
	void GLTransformToCameraCoords() const;

	// Configure a line widget to match a line detection
	/*void ConfigureLineWidget(LineWidget& w,
	                         const LineDetection& det,
	                         int index,
	                         bool visible,
	                         bool add_events,
	                         const string& label);*/
	// Recreate the widgets for the line segments in this keyframe
	//void ConfigureLineWidgets();
	// Configure the widgets that comprise the border
	void ConfigureBorder();

	// Add a line widget for a line in retina coordinates
	LineWidget& AddLineInRetina(const Vec3& ret_a,
	                            const Vec3& ret_b,
	                            const float width,
	                            const PixelRGB<byte>& color);

	// Transform a point from undistorted image coordinates to the world
	Vec3 ImagePtToWorld(const Vec2& retinaPt);
	// Project a world point to the retina (but still in world coords)
	Vec3 WorldToRetina(const Vec3& worldPt);
};


// Represents a set of map points. We have a single widget for all
// the points rather than seperate widgets for each point because we
// need to render all the points inside a single glBegin/glEnd (for
// efficiency sake).
class PointCloudWidget : public Widget3D {
public:
	// The list of points
	const vector<Vec3>& points;
	// The index of the selected point
	int selected_point;
	// Event fired when the selected point changes
	Event<> SelectedPointChanged;
	// Screen locations of the points. Updated in OnRender iff
	// selectable() is true
	vector<Vec2 > screen_pts;
	// Initialize the widget
	PointCloudWidget(const vector<Vec3>& points);
	// Draw the map points as GL_POINTs
	void OnRender();
	// Handle clicks
	void OnClick(int button, const Vec2& mouse);
	// Return true if the mouse is over a map point
	bool HitTest(const Vec2& mouse) const;
	// Get the point at the given screen position, or -1 if there
	// isn't one.
	int GetPointAt(const Vec2& mouse) const;
};


// Represents a map and its vizualization
class MapWidget : public Widget3D {
public:
	// The map
	const Map* map;
	// The keyframe widgets. Memory is owned by Viewer3D.
	vector<KeyFrameWidget*> kf_widgets;
	// The point cloud widget. Memory is owned by Viewer3D.
	PointCloudWidget* pts_widget;
	// The ground plane grid.
	GroundPlaneWidget* ground_plane;

	// Constructor
	MapWidget();
	MapWidget(const Map* themap);
	// Attach to a map
	void Configure(const Map* themap);
	// Register a newly added keyframe and add a widget for it
	KeyFrameWidget& RegisterKeyFrame(const KeyFrame& kf);
	// Bind keys in the viewer
	void BindKeys();
	// Change the retina plane z pos
	void SetRetinaPos(double z);
	// Increment/decrement the retina plane z pos
	void ChangeRetinaPos(int delta);
	// Show/hide detected lines in keyframes
	//void ToggleLines();
};
}
