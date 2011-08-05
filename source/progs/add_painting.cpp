#include <iomanip>

#include <LU.h>
#include <so3.h>

#include "entrypoint_types.h"
#include "map_widgets.h"
#include "map.h"
#include "map_io.h"
#include "map.pb.h"
#include "textons.h"
#include "vars.h"
#include "bld_helpers.h"
#include "floorplan_renderer.h"

#include "widget3d.h"
#include "viewer3d.h"
#include "texture_manager.h"

#include "gl_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

toon::Matrix<3,2> GetMatForAxis(int i) {
	toon::Matrix<3,2> m = toon::Zeros;
	m[i == 2 ? 0 : 1-i][0] = 1;
	m[i == 2 ? 1 : 2][1] = 1;
	return m;
}

class PaintingViewer {
public:
	GlutWindow window_;
	TextureManager textures_;
	const Map& map_;
	const proto::FloorPlan& floorplan_;
	const ImageBundle* painting_;
	Vec2 painting_size_;  // Size of painting in the world
	Vec3 painting_center_; // Position of painting in world

	int currentFrame_;
	Polygon<4> quad_;
	int currentWall_;
	Vec3 drag_offset_;

	FloorPlanRenderer re_;

	PaintingViewer(const Map& map,
								 const proto::FloorPlan& floorplan,
								 const ImageBundle& painting);
	void Run();
	void SetCurrentFrame(int index);
	void SetPaintingCenter(const Vec3& c, int orient);

	void window_Display();
	void window_MouseMove(const Vec2& mousePt);
	void window_MouseDown(int button, const Vec2& mousePt);
	void window_MouseDrag(int button, const Vec2& mousePt);
	void window_SpecialKeyDown(int key);
};

PaintingViewer::PaintingViewer(const Map& map,
															 const proto::FloorPlan& floorplan,
															 const ImageBundle& painting)
	: map_(map),
		floorplan_(floorplan),
		painting_(&painting),
		currentFrame_(0),
		currentWall_(-1) {
	SetCurrentFrame(0);
	window_.Display.add(bind(&PaintingViewer::window_Display, this));
	window_.MouseMove.add(bind(&PaintingViewer::window_MouseMove, this, _1));
	window_.MouseDown.add(bind(&PaintingViewer::window_MouseDown, this, _1, _2));
	window_.MouseDrag.add(bind(&PaintingViewer::window_MouseDrag, this, _1, _2));
	window_.SpecialKeyDown.add
		(bind(&PaintingViewer::window_SpecialKeyDown, this, _1));

	// Compute size of the painting in the world
	double room_height = floorplan_.zceil() - floorplan_.zfloor();
	double long_len = room_height / 2.5;
	if (painting_->nx() > painting_->ny()) {
		painting_size_[0] = long_len;
		painting_size_[1] = 1. * painting_->ny() * painting_size_[0] / painting_->nx();
	} else {
		painting_size_[1] = long_len;
		painting_size_[0] = 1. * painting_->nx() * painting_size_[1] / painting_->ny();
	}	
}

void PaintingViewer::Run() {
	window_.Run();
}

void PaintingViewer::SetCurrentFrame(int index) {
	currentFrame_ = index;
	re_.Configure(map_.frames[index].image.pc());
	re_.Render(floorplan_);
	window_.Invalidate();
}

void PaintingViewer::SetPaintingCenter(const Vec3& c, int orient) {
	toon::Matrix<3,2> m = GetMatForAxis(orient);
	painting_center_ = c;
	double xlen = painting_size_[0];
	double ylen = painting_size_[1];
	quad_.verts[0] = c + m*makeVector(-xlen/2, -ylen/2);
	quad_.verts[1] = c + m*makeVector(-xlen/2, ylen/2);
	quad_.verts[2] = c + m*makeVector(xlen/2, ylen/2);
	quad_.verts[3] = c + m*makeVector(xlen/2, -ylen/2);
	window_.Invalidate();
}

void PaintingViewer::window_Display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	const Frame& frame = map_.frames[currentFrame_];
	if (!frame.image.loaded()) {
		const_cast<Frame&>(frame).LoadImage();
	}

	// Draw the image
	RenderFullScreen(textures_.LoadOrLookup(&frame.image));

	// Draw the painting
	WITHOUT(GL_DEPTH_TEST) {
		ConfigureGLForCamera(frame.image.pc());
		textures_.Select(painting_);
		WITH(GL_TEXTURE_2D) GL_PRIMITIVE(GL_QUADS) {
			glTexCoord2f(0,1);
			glVertexV(quad_.verts[0]);
			glTexCoord2f(0,0);
			glVertexV(quad_.verts[1]);
			glTexCoord2f(1,0);
			glVertexV(quad_.verts[2]);
			glTexCoord2f(1,1);
			glVertexV(quad_.verts[3]);
		}
		glError();
	}
}

void PaintingViewer::window_MouseDrag(int button, const Vec2& mousePt) {
	int orient = re_.GetOrientationAt(mousePt);
	if (orient != kVerticalAxis) {
		int idx = re_.GetWallIndexAt(mousePt);
		if (idx == currentWall_) {
			Vec3 pt = re_.BackProject(mousePt);
			SetPaintingCenter(pt+drag_offset_, orient);
		}
	}
}

void PaintingViewer::window_MouseDown(int button, const Vec2& mousePt) {
	int orient = re_.GetOrientationAt(mousePt);
	if (orient != kVerticalAxis) {
		Vec3 worldPt = re_.BackProject(mousePt);
		if (currentWall_ == -1) {
			currentWall_ = re_.GetWallIndexAt(mousePt);
			SetPaintingCenter(worldPt, orient);
			drag_offset_ = makeVector(0,0,0);
			window_.Invalidate();
		} else {
			drag_offset_ = painting_center_ - worldPt;
		}
	}
}

void PaintingViewer::window_MouseMove(const Vec2& mousePt) {
}

void PaintingViewer::window_SpecialKeyDown(int key) {
	int nf = map_.frames.size();
	switch (key) {
	case GLUT_KEY_LEFT:
		SetCurrentFrame((currentFrame_ + nf - 1) % nf);
		break;
	case GLUT_KEY_RIGHT:
		SetCurrentFrame((currentFrame_ + 1) % nf);
		break;
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);

	po::options_description desc("Allowed options");
	desc.add_options()
    ("help", "produce help message")
		("sequence", po::value<string>()->required(), "Map by sequence name")
		("painting", po::value<string>(), "Painting image")
		;

	// Parse options
	po::variables_map opts;
	try {
		po::store(po::parse_command_line(argc, argv, desc), opts);
		po::notify(opts);
	} catch (const po::required_option& ex) {
		cout << "Missing required option: "
				 << ex.get_option_name() << "\n" << desc << "\n";
		return 1;
	}
	if (opts.count("help")) {
    cout << desc << "\n";
    return -1;
	}

	ImageBundle painting(opts["painting"].as<string>());
	ResetAlpha(painting.rgb);

	//WriteImage("out/foo.png", painting.rgb);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	string path = GetMapPath(opts["sequence"].as<string>());
	LoadXmlMapWithGroundTruth(path, map, gt_map);

	PaintingViewer v(map, gt_map.floorplan(), painting);
	v.Run();

	return 0;
}
