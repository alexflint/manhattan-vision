#include <unistd.h>

#include <vector>
#include <list>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <google/heap-profiler.h>

#include <mex.h>

#include <VW/Image/imagecopy.h>

#include "common_types.h"
#include "manhattan_dp.h"
#include "map.h"
#include "map.pb.h"
#include "matlab_utils.h"
#include "manhattan_inference.h"
#include "bld_helpers.h"

#include "counted_foreach.tpp"

using namespace std;
using namespace indoor_context;
using boost::format;

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	InitMex();
	if (nlhs != 1 || nrhs == 0) {
		mexErrMsgTxt("Usage: cases=dp_load_cases('set1', 'set2', ...)\n");
	}

	// Load the data
	format case_tpl("%s:%d");
	vector<string> cases;
	fs::path sequences_dir("sequences");

	// Feature generator (placed here to allow memory to be shared
	// across multiple invokations)
	LineSweepFeatureGenerator feature_gen;

	for (int i = 0; i < nrhs; i++) {
		string sequenceName = MatlabArrayToString(prhs[i]);

		// Compute features for each frame
		Map& map = FrameStore::instance().GetMap(sequenceName);
		const proto::FloorPlan& fp = FrameStore::instance().GetFloorPlan(sequenceName);

		//for (int i = 0; i < 5; i++) {  // temp hack
		//KeyFrame& kf = map.kfs[i];  // temp hack
		BOOST_FOREACH(KeyFrame& kf, map.kfs) {
			string case_name = str(case_tpl % sequenceName % kf.id);
			DLOG << "Initializing case " << case_name;

			// Generate features
			kf.LoadImage();
			feature_gen.Compute(kf.image);

			// Initialize the frame data
			ManhattanInference& inf = FrameStore::instance().Get(case_name);
			inf.Prepare(kf.image, fp, feature_gen.features);

			// Add the case name
			cases.push_back(case_name);
		}
	}

	// Create a struct array
	DLOG << "Loaded " << cases.size() << " training cases";
	mwSize nCases = cases.size();
	const char *fieldNames[] = {"name", "gt_orients"};
	plhs[0] = mxCreateStructArray(/*num dimensions*/ 1, &nCases,
																/*num fields*/ 2, fieldNames);
	int nameField = mxGetFieldNumber(plhs[0], "name");
	int orientsField = mxGetFieldNumber(plhs[0], "gt_orients");

	// Copy the feature matrices in
	for (int i = 0; i < cases.size(); i++) {
		mxSetFieldByNumber(plhs[0], i, nameField,
											 NewMatlabArrayFromString(cases[i]));
		ManhattanInference& inf = FrameStore::instance().Get(cases[i]);
		mxSetFieldByNumber(plhs[0], i, orientsField,
											 NewMatlabArrayFromMatrix(inf.gt_labels));
	}
}
