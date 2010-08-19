#include <exception>
#include <mex.h>

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]);

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
	try {
		_mexFunction(nlhs, plhs, nrhs, prhs);
	} catch (const std::exception& ex) {
		mexPrintf("Caught exception:\n%s", ex.what());
	}
}
