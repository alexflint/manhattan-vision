#include <unistd.h>

#include <vector>
#include <list>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <mex.h>

#include <VW/Image/imagecopy.h>

#include "common_types.h"
#include "vars.h"
#include "manhattan_dp.h"
#include "map.h"
#include "map.pb.h"
#include "matlab_utils.h"

#include "counted_foreach.tpp"

using namespace std;
using namespace indoor_context;
using boost::format;

struct TrainingCase {
	mxArray* features;
	mxArray* name;
};

static const string kWorkingDir = "/homes/50/alexf/work/indoor_context/build";  // ugly hack

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	if (chdir(kWorkingDir.c_str())) {  // ugly hack!
		mexPrintf("Could not chdir to %s\n", kWorkingDir.c_str());
		mexErrMsgTxt("Could not chdir, exiting.");
	}

	InitVars();
	if (nlhs != 1 || nrhs == 0) {
		mexErrMsgTxt("Usage: data=load_training_data('set1', 'set2', ...)\n");
	}


	ManhattanDPFeatures features;

	format pathTpl("sequences/%s/ground_truth/truthed_map.pro");
	format nameTpl("%s:%d");
	ptr_vector<TrainingCase> cases;


	for (int i = 0; i < nrhs; i++) {
		// Get the path
		string sequenceName = MatlabArrayToString(prhs[i]);
		string path = str(pathTpl % sequenceName);
		mexPrintf("Loading map from %s\n", path.c_str());

		// Load the map
		Map map;
		proto::TruthedMap gt_map;
		map.LoadWithGroundTruth(path, gt_map);

		// Generate features for each frame
		COUNTED_FOREACH(int j, KeyFrame& kf, map.kfs) {
			cases.push_back(new TrainingCase);
			TrainingCase& c = cases.back();
			c.name = StringToMatlabArray(str(nameTpl % sequenceName % j));

			//mexPrintf("Generating features for keyframe %d of %d\n", j, map.kfs.size());
			//kf.LoadImage();
			//Mat3 floorToCeil = GetFloorCeilHomology(kf.image.pc(), gt_map.floorplan());
			//features.ComputeFeatures(kf.image, floorToCeil);
			//c.features = NewMatlabArrayFromTable(features.features);
			//mwSize dims[2] = { 10, 10 };
			//c.features = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
			//kf.UnloadImage();
		}
	}

	// Create a struct array
	cout << "Loaded " << cases.size() << " training cases\n";
	mwSize nCases = cases.size();
	const char *fieldNames[] = {"name"};
	plhs[0] = mxCreateStructArray(1, &nCases, 1, fieldNames);
	int nameField = mxGetFieldNumber(plhs[0], "name");

	// Copy the feature matrices in
	for (int i = 0; i < cases.size(); i++) {
		mxSetFieldByNumber(plhs[0], i, nameField, cases[i].name);
	}
}
