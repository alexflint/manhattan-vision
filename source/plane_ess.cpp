#include <iomanip>

#include <LU.h>

#include "common_types.h"
#include "map_widgets.h"
#include "map.h"
#include "textons.h"
#include "vars.h"
#include "viewer3d.h"
#include "gl_utils.tpp"

using namespace indoor_context;

void ToggleTextons(MapViz& mapviz) {
	BOOST_FOREACH(KeyFrameWidget* w, mapviz.kf_widgets) {
		w->show_textons = !w->show_textons;
	}
	mapviz.viewer.Invalidate();
}

Matrix<3> GetAxisPerm(int axis, double plane_offset) {
	int other1 = (axis+1)%3;
	int other2 = (axis+2)%3;
	Matrix<3> perm = Zeros;
	perm[0][other1] = 1;
	perm[1][other2] = 1;
	perm[2][axis] = plane_offset;
	return perm;
}

// Represents an axis-aligned box in 2D
template <typename T=double>
class Bounds2d {
	T top_, bottom_, left_, right_;
public:
	// Initialize to empty bounds at origin
	Bounds2d() : top_(0), bottom_(0), left_(0), right_(0) { }
	// Initialize to provided top, left, bottom, right
	Bounds2d(const T& t, const T& b, const T& l, const T& r)
		: top_(t), bottom_(b), left_(l), right_(r) { }
	// Initialize to given top-left and set width and height
	Bounds2d(Vector<2,T> tl, const T& w, const T& h) : top_(tl[1]),
																										 bottom_(tl[1]+h),
																										 left_(tl[0]),
																										 right_(tl[0]+w) {
	}
	// Initialize to smallest bounds containing these two points
	Bounds2d(const Vector<2,T>& a, const Vector<2,T>& b) : top_(min(a[1], b[1])),
																												 bottom_(max(a[1], b[1])),
																												 left_(min(a[0], b[0])),
																												 right_(max(a[0], b[0])) {
	}
																						 
	// Accessors
	inline T top() const { return top_; }
	inline T left() const { return left_; }
	inline T bottom() const { return bottom_; }
	inline T right() const { return right_; }

	// Mutators
	inline void set_top(T v) { top_ = v; }
	inline void set_bottom(T v) { bottom_ = v; }
	inline void set_left(T v) { left_ = v; }
	inline void set_right(T v) { right_ = v; }

	// Derived quantities
	inline T width() const { return right_ - left_; }
	inline T height() const { return bottom_ - top_; }
	Vector<2,T> top_left() const { return makeVector(left_, top_); }
	Vector<2,T> top_right() const { return makeVector(right_, top_); }
	Vector<2,T> bottom_left() const { return makeVector(left_, bottom_); }
	Vector<2,T> bottom_right() const { return makeVector(right_, bottom_); }
	inline T centerX() const { return (left_+right_)/2; }
	inline T centerY() const { return (top_+bottom_)/2; }
	Vector<2> center() const { return makeVector(centerX(),centerY()); }

	// Tests
	bool contains(const Vector<2>& v) const {
		return v[0] >= left_ && v[0] <= right_ && v[1] >= top_ && v[1] <= bottom;
	}
};

class SizeableQuad : public Widget3D {
public:
	Matrix<3> perm_;
	Bounds2d<> region_;
	Vector<4> plane_eqn_;
	QuadWidget quad;
	LineWidget guides[4];

	Event Resize;

	SizeableQuad() {
		Initialize();
	}

	void Initialize() {
		Add(quad);
		quad.Hide();
		for (int i = 0; i < 4; i++) {
			Add(guides[i]);
			guides[i].width = 2.5;
			guides[i].color = quad.color;
			guides[i].selectColor = PixelRGB<float>(1,1,1);
			guides[i].SetSelectable(true);
			guides[i].SetDraggable(true);
			guides[i].MouseDrag.add(bind(&SizeableQuad::Guide_MouseDrag, this, i));
			guides[i].Hide();
		}
	}

	void Configure(int axis, double plane_offset, const Bounds2d<>& region) {
		region_ = region;
		perm_ = GetAxisPerm(axis, plane_offset);

		plane_eqn_ = makeVector(0, 0, 0, -plane_offset);
		plane_eqn_[axis] = 1;

		quad.Show();
		ConfigureQuad();

		for (int i = 0; i < 4; i++) {
			guides[i].Show();
		}
	}

	void Guide_MouseDrag(int index) {
		Vector<3> worldPt = ScreenToWorld(viewer().mouse_location());
		Vector<2> projPt = (perm_*worldPt).slice<0,2>();
		switch (index) {
		case 0: // left
			region_.set_left(projPt[0]);
			break;
		case 1: // right
			region_.set_right(projPt[0]);
			break;
		case 2: // top
			region_.set_top(projPt[1]);
			break;
		case 3: // bottom
			region_.set_bottom(projPt[1]);
			break;
		}

		Resize.fire();
		ConfigureQuad();
	}		

	void ConfigureQuad() {
		// quad
		quad.a = perm_.T() * unproject(region_.top_left());
		quad.b = perm_.T() * unproject(region_.bottom_left());
		quad.c = perm_.T() * unproject(region_.bottom_right());
		quad.d = perm_.T() * unproject(region_.top_right());
		// left guide
		guides[0].a = quad.a;
		guides[0].b = quad.b;
		// right guide
		guides[1].a = quad.c;
		guides[1].b = quad.d;
		// top guide
		guides[2].a = quad.a;
		guides[2].b = quad.d;
		// bottom guide
		guides[3].a = quad.b;
		guides[3].b = quad.c;
	}

	Vector<3> ScreenToWorld(const Vector<2>& mousePt) {
		double height = viewer().windowSize.y;
		Vector<2> viewPt = makeVector(mousePt[0], height-mousePt[1]);  // invert Y coordinate
		Vector<2> eyePt = viewer().WindowToViewport(viewPt);
		Matrix<4> A = GetGLProjection() * GetGLModelView();
		A.slice<2,0,1,4>() = plane_eqn_.as_row();
		Vector<4> worldPt = LU<>(A).backsub(makeVector(eyePt[0], eyePt[1], 0, 1));
		return project(worldPt);
	}
};




int prev_pt;
int axis;
vector<LineWidget*> lines;

void PointCloud_SelectedPointChanged(const MapViz& mapviz,
																		 SizeableQuad& quad) {
	int pti = mapviz.pts_widget->selected_point;
	if (pti == prev_pt) {
		axis = (axis+1)%3;
	} else {
		axis = 0;
	}
	prev_pt = pti;
		
	Vector<3> v = mapviz.map.pts[pti];
	double plane_offset = v[axis];

	Matrix<3> perm = GetAxisPerm(axis, plane_offset);
	Vector<2> c = (perm*v).slice<0,2>();
	Bounds2d<> region(c-makeVector(1,1), c+makeVector(1,1));
	quad.Show();

	quad.Configure(axis, plane_offset, region);
}


void DrawQuadInKfs(const MapViz& mapviz, SizeableQuad& quad) {
	GL_PRIMITIVE(GL_LINES) {
		BOOST_FOREACH(KeyFrameWidget* kfw, mapviz.kf_widgets) {
			for (int j = 0; j < 4; j++) {
				const LineWidget& line = quad.guides[j];
				glVertexV(kfw->WorldToRetina(line.a));
				glVertexV(kfw->WorldToRetina(line.b));
			}
		}
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map and rotate to canonical frame
	MapViz mapviz;
	mapviz.map.RotateToSceneFrame();

	// Compute the texton maps
	TextonVocab vocab;
	vocab.Load(GV3::get<string>("Textons.VocabFile"));
	BOOST_FOREACH(KeyFrame& kf, mapviz.map.kfs) {
		kf.ComputeTextonMap(vocab);
	}
	mapviz.viewer.BindKey('t', bind(&ToggleTextons, ref(mapviz)));


	// Add the quad
	SizeableQuad quad;
	mapviz.viewer.Add(quad);

	mapviz.viewer.Add(bind(&DrawQuadInkfs, ref(mapviz), ref(quad)));

	// Hook the quad the the point cloud
	axis = 0;
	prev_pt = -1;
	mapviz.pts_widget->SelectedPointChanged.add
		(bind(&PointCloud_SelectedPointChanged, ref(mapviz), ref(quad)));

	// Enter the event loop
	mapviz.Run();

	return 0;
}
