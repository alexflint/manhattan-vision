#include <iomanip>
#include <map>

#include <GL/glut.h>

#include "common_types.h"
#include "map.h"
#include "map_widgets.h"
#include "glut_window.h"
#include "viewer3d.h"

#include "map.pb.h"

namespace indoor_context {
	// Represents a state of the floorplan UI
	namespace UIState {
		enum {
			VIEW, INSERT, MODIFY
		};
	};

	// State of the camera view UI
	namespace ViewType {
		enum {
			WIREFRAME, FILLED
		};
	};

	// Responsible for manipulating floorplans
	class FloorPlanEditor {
	public:
		proto::FloorPlan* floorplan;
		scoped_ptr<proto::FloorPlan> mem_floorplan;  // used to manage memory of above
		int uiState;
		int mouseState;
		int hoverItem, selectedItem, dragAxis;
		double dragOffset;
		bool enableSnap;
		toon::Vector<2> hoverPt, endPt;
		const Map& map;

		double sideDragOffset;
		int sideDragItem, sideHoverItem;

		int selectedFrameIndex_;  // index of the key frame currently visible in the camera view
		int camViewState;  // whether the camera view window shows wireframe or filled models

		MapWidget mapWidget;  // Manages drawing points and keyframes in GL
		Viewer3D planView;  // Window to draw out the floorplan itself
		Viewer3D sideView;  // Window to control the floor and ceil pos
		Viewer3D freeView;  // Window displaying map+floorplan, can rotate/pan freely
		GlutWindow camView;  // Window displaying the floorplan projected into frames
		TextureManager camTextures;

		// Last filename loaded from or saved to
		string filename_;

		// Map from keys to event handlers
		std::map<int, Event<> > keyMap_;

		// Fired when a key is pressed in any of the three windows
		Event<void(byte c)> KeyDown;
		inline Event<>& KeyStroke(int key) { return keyMap_[key]; }

		// Initialize a floorplan editor for the specified map
		FloorPlanEditor(const Map& themap);

		void ResetInternal();

		// These are the main public methods that should be used from outside
		void Run();  // enter the GLUT loop
		void Invalidate();  // redraw all windows
		void ResetView();   // reset the camera viewpoints
		void SetUIState(int state);  // set the interaction mode to one of the UIState enum values
		void ToggleUIState();
		void ToggleCamViewState();  // toggle between wireframe and filled mode

		// Manipulate mouse coordinates
		toon::Vector<2> MouseToCanvas(const toon::Vector<2>& mousePt);
		double MouseToHeight(const toon::Vector<2>& mousePt);
		toon::Vector<2> Snap(const toon::Vector<2>& v, int snap_axis, int ignore=-1);

		// Render the floorplan. Parameter is one of the ViewType enum values.
		void RenderFloorPlan(int viewType);
		void RenderPlanView();  // the top-down view
		void RenderSideView();  // the side-on view
		void RenderCamView();   // the camera frame view
		void RenderFreeView();  // the free zooming, rotating view

		// Select the frame to display in the camView window
		void SelectFrameByIndex(int index);

		// Handlers for the map widget
		void mapWidget_PreRender();

		// Handlers for the plan view
		void planView_MouseDown(int button, const toon::Vector<2>& mousePt);
		void planView_MouseDrag(int button, const toon::Vector<2>& mousePt);
		void planView_MouseMove(const toon::Vector<2>& mousePt);

		// Handlers for the side view
		void sideView_MouseDown(int button, const toon::Vector<2>& mousePt);
		void sideView_MouseUp(int button, const toon::Vector<2>& mousePt);
		void sideView_MouseDrag(int button, const toon::Vector<2>& mousePt);
		void sideView_MouseMove(const toon::Vector<2>& mousePt);

		// Handlers for the camera view
		void camView_Display();

		// Handler for keystrokes in all windows
		void anyView_KeyDown(byte c);
		// Handler for special keys (arrows, function keys, etc) in all windows
		void anyView_SpecialKeyDown(int key);

		// Attach to and begin editing a floorplan
		void Attach(proto::FloorPlan* floorplan);
		// Attach to a floor plan and take ownership of its memory
		void AttachAndManage(proto::FloorPlan* floorplan);

		// Read and write floorplans
		void Load(const string& filename);
		void Write(const string& filename);
	};
}
