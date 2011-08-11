#include <mex.h>
#include <string>

void mexFunction(int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[]) {
  mexPrintf("Hello, world\n");
	std::string test("hello");
}
