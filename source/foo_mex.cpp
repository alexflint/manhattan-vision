/*
 * foo_mex.cpp
 *
 *  Created on: 21 Jul 2010
 *      Author: alexf
 */

//#include <string>

#include <boost/filesystem.hpp>

#include <mex.h>

#include "common_types.h"
#include "vars.h"
#include "manhattan_dp.h"
#include "discriminative_manhattan_dp.h"
#include "map.h"
#include "map.pb.h"

//#include <VW/Image/imagergb.h>

using namespace indoor_context;

using namespace std;

int count() {
	static int a = 1;
	return a++;
}

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	InitVars();
	mexPrintf("This has been called %d times\n", count());

	Map map;
	proto::TruthedMap gt_map;
	DiscriminativeManhattanDP recon;

	string s;
	//google::protobuf::io::StringOutputStream sos(&s);
	mexPrintf("Done\n");
}
