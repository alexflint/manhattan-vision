#include <stdlib.h>
#include <exception>
#include <mex.h>
#include <google/profiler.h>

#include "matlab_utils.h"

#include "log.tpp"

// name of the environment variable that activates profiling of MEX functions
static const char* kMexProfileVar = "MEXCPUPROFILE";

// Forward declaration of the entry function that is implemented in each .mex.cpp file
void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]);

// The MEX entry point
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
	try {
		const char* cpuprofile = getenv(kMexProfileVar);
		if (cpuprofile != NULL) {
			ProfilerStart(cpuprofile);
		}
		TITLED("Executing MEX function: " << mexFunctionName()) {
			indoor_context::InitMex();
			_mexFunction(nlhs, plhs, nrhs, prhs);
		}
		if (cpuprofile != NULL) {
			ProfilerStop();
		}
	} catch (const std::exception& ex) {
		mexPrintf("Caught exception:\n%s", ex.what());
	}
}
