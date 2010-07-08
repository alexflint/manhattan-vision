#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"



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

	// Get the frames
	KeyFrame& kf1 = *map.KeyFrameByIdOrDie(frame1_id);
	KeyFrame& kf2 = *map.KeyFrameByIdOrDie(frame2_id);
	Matrix<3,4> m1 = kf1.pc->GetLinearApproximation();
	Matrix<3,4> m2 = kf2.pc->GetLinearApproximation();

	return 0;
}
