// TODO: copied this from plane_ess.cpp so it wouldn't get lost. Need
// to refactor a bit to make it work.


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
