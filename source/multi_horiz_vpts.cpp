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

void Line_Click(MapViz& mapviz, int kf_index, int line_index) {
	const KeyFrame& kf = mapviz.map.kfs[kf_index];
	const LineSegment& seg = kf.vpts.lines.segments[line_index];

	Vector<3> horiz_vline = makeVector(0,0,1);

	Matrix<3> line_to_retina = LU<>(kf.unwarped.image_to_retina).get_inverse().T()
	Vector<3> retline = line_to_retina * fromVNL(seg.line);

	Vector<3> wline = kf.WfC.get_rotation() * retline;

	Vector<3> world_line = fromVNL(seg.line_cond);
	Vector<3> scene_line = mapviz.map.world_rotation.inverse() * world_line;


	Vector<3> horiz_vpt = wline ^ horiz_vline;

	DREPORT(wline, horiz_vline, horiz_vpt);

	Vector<3> cam_vpt = kf.CfW.get_rotation() * horiz_vpt;

	DREPORT(cam_vpt);
	DREPORT(cam_vpt*retline);

	Vector<3> world_endpt = kf.WfC * atretina(cam_vpt);
	Vector<3> world_origin = kf.WfC * makeVector(0,0,0);

	mapviz.viewer.AddOwned(new LineWidget(world_origin, world_endpt, 3.5));
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

	// Add listeners
	COUNTED_FOREACH(int kfi, KeyFrameWidget* kf, mapviz.kf_widgets) {
		kf->ConfigureLineWidgets();
		COUNTED_FOREACH(int segi, LineWidget* w, kf->line_widgets) {
			w->Click.add(bind(&Line_Click, ref(mapviz), kfi, segi));
		}
	}

	// Count the number of lines
	/*int num_lines = 0;
	BOOST_FOREACH(const KeyFrame& kf, mapviz.map.kfs) {
		num_lines += kf.vpts.lines.segments.size();
	}

	// Vanishing line of the XY plane
	Vector<3> horiz_vline = makeVector(0,0,1);*/

	// Look for further vpts
	/*const int kSamples = 20;
	for (int i = 0; i < kSamples; i++) {
		// Choose a random line
		int pivot_line = rand() % num_lines;
		int kfi = 0;
		while (pivot_line < mapviz.map.kfs[kfi].lines.segments.size()) {
			kfi++;
			pivot_line -= mapviz.map.kfs[kfi].lines.segments.size();
		}
		const KeyFrame& kf = mapviz.map.kfs[kfi];
		const LineSegment& seg = kf.lines.segments[pivot_line];

		const Vector<3> world_line = fromVNL(seg.line_cond);
		const Vector<3> scene_line = map.world_rotation.inverse() * world_line;

		const Vector<3> horiz_vpt = scene_line ^ horiz_vline;*/

		
		

	// Enter the event loop
	mapviz.Run();

	return 0;
}
