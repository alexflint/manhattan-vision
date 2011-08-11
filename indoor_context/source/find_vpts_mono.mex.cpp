#include <string>

#include <boost/format.hpp>

#include "common_types.h"
#include "image_utils.h"
#include "map.h"
#include "map.pb.h"
#include "vanishing_points.h"

using namespace indoor_context;
using namespace toon;
using boost::format;

lazyvar<float> gvWallPenalty("ManhattanDP.DefaultWallPenalty");
lazyvar<float> gvOcclusionPenalty("ManhattanDP.DefaultOcclusionPenalty");

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	if (nrhs != 2) {
		mexErrMsgTxt("Usage: orients=dp_solve(case, objective)\n");
	}
	ConstMatlabStructure frame = FrameProto.From(prhs[0]);



}
