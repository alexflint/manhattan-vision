#include "mex.h" 

class Foo {
 public:
	static void blah() {
		mexPrintf("Hello, C++ world!\n"); 
	}
};

void mexFunction(int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[]) {
	FOO::blah();
  mexPrintf("Hello, world!\n"); 
}
