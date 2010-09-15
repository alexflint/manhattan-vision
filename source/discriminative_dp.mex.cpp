#include <unistd.h>

#include <mex.h>

#include <VW/Image/imagecopy.h>

#include "common_types.h"
#include "vars.h"
#include "manhattan_dp.h"
#include "manhattan_inference.h"
#include "map.h"
#include "map.pb.h"
#include "bld_helpers.h"

namespace indoor_context {

static const string kMapPath = "sequences/lab_kitchen1/ground_truth/truthed_map.pro";
static const int kFrameIndex = 24;

class DiscriminativeDPDriver {
public:
	Map map;
	proto::TruthedMap gt_map;
	ManhattanInference recon;

	bool initialized;

	DiscriminativeDPDriver() : initialized(false) {
	}

	void Initialize() {
	}

	void Predict() {
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
