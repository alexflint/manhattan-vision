#include <unistd.h>

#include <mex.h>

#include <VW/Image/imagecopy.h>

#include "common_types.h"
#include "vars.h"
#include "manhattan_dp.h"
#include "discriminative_manhattan_dp.h"
#include "map.h"
#include "map.pb.h"

namespace indoor_context {

static const string kMapPath = "sequences/lab_kitchen1/ground_truth/truthed_map.pro";
static const int kFrameIndex = 24;

class DiscriminativeDPDriver {
public:
	Map map;
	proto::TruthedMap gt_map;
	ManhattanDPFeatures recon;

	bool initialized;

	DiscriminativeDPDriver() : initialized(false) {
	}

	void Initialize() {
		map.LoadWithGroundTruth(kMapPath, gt_map);
		KeyFrame& kf = *map.KeyFrameByIdOrDie(kFrameIndex);
		kf.LoadImage();  // leave the image loaded
		Mat3 floorToCeil = GetFloorCeilHomology(*kf.pc, gt_map.floorplan());
		recon.ComputeFeatures(kf.image, floorToCeil);
		//kf.UnloadImage();  leave the image loaded because we will use it later
	}

	void Predict() {
		toon::Vector<> w(18);
		w[3] = w[10] = w[17] = 1.0;
		recon.ComputeScores(w);
	}
};

}

using namespace indoor_context;

scoped_ptr<DiscriminativeDPDriver> driver;

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	cerr << "Current working dir: " << get_current_dir_name() << endl;
	if (!driver) {
		InitVars();
		driver.reset(new DiscriminativeDPDriver);
		driver->Initialize();
	}
	driver->Predict();
}
