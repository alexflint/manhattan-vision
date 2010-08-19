#include <LU.h>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"

#include "camera.h"
#include "colors.h"
#include "canvas.h"
#include "floor_ceil_map.h"

#include "vector_utils.tpp"
#include "math_utils.tpp"

using indoor_context::Sign;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 4) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro INDEX1 INDEX2";
		return 0;
	}

	// Input arguments
	const char* path = argv[1];
	int frame1_id = atoi(argv[2]);
	int frame2_id = atoi(argv[3]);

	// Load the map
	Map map;
	proto::TruthedMap tru_map;
	map.LoadWithGroundTruth(path, tru_map);

	// Get the floor and ceiling positions
	double zfloor = tru_map.floorplan().zfloor();
	double zceil = tru_map.floorplan().zceil();
	Vec3 vup = map.kfs[0].pc->pose.inverse() * makeVector(0,1,0);
	if (Sign(zceil-zfloor) == Sign(vup[2])) {
		swap(zfloor, zceil);
	}

	// Load the key frames
	KeyFrame& kf1 = *map.KeyFrameByIdOrDie(frame1_id);
	KeyFrame& kf2 = *map.KeyFrameByIdOrDie(frame2_id);
	kf1.LoadImage();
	kf2.LoadImage();

	// Get the homographies
	Mat3 hfloor = GetHomographyVia(*kf1.pc, *kf2.pc, makeVector(0, 0, -1, zfloor));
	Mat3 hceil = GetHomographyVia(*kf1.pc, *kf2.pc, makeVector(0, 0, -1, zceil));
	Mat3 floorToCeil = GetManhattanHomology(*kf1.pc, zfloor, zceil);
	Vec3 horizon = kf1.pc->GetImageHorizon();

	// Compute integral col image for kf2
	MatF aux_orients;
	GetTrueOrients(tru_map.floorplan(), *kf2.pc, aux_orients);
	IntegralColImage<3> integ_orients(aux_orients);

	// Transfer some quads
	int x = 100;
	int y = 45;
	int axis = 1;

	double opp_x



	// Draw some points
	BrightColors bc;
	FileCanvas canvas1("out/from_pts.png", kf1.image.rgb);
	FileCanvas canvas2("out/to_pts.png", kf2.image.rgb);
	for (int y = 0; y < kf1.image.ny(); y += 100) {
		for (int x = 0; x < kf2.image.nx(); x += 100) {
			Vec3 p = makeVector(x,y,1.0);
			if (horizon*p <= 0) continue;  // check that point is below horizon

			Vec3 q = floor2ceil * p;

			PixelRGB<byte> color = bc.Next();
			canvas1.DrawDot(project(p), 4.0, Colors::white());
			canvas1.DrawDot(project(p), 3.0, color);
			canvas1.DrawDot(project(q), 3.0, color);
			canvas1.StrokeLine(project(p), project(q), color);

			canvas2.DrawDot(project(hfloor*p), 4.0, Colors::white());
			canvas2.DrawDot(project(hfloor*p), 3.0, color);
			canvas2.DrawDot(project(hceil*q), 3.0, color);
			canvas2.StrokeLine(project(hfloor*p), project(hceil*q), color);
		}
	}

	return 0;
}
