#include <mex.h>

#include "common_types.h"

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	DLOG << "foo";
	int x = 3;
	DREPORT(x);
	DLOG << "bar\nbar\n";
}
