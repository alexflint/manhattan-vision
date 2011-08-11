#include <iostream>

#include <mex.h>

using namespace std;

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]);

int main(int argc, char **argv) {
	if (argc == 1) {
		cout << "Usage: "<<argv[0]<<" <num output params> <input1> <input2> ..." << endl;
		return 0;
	}

	int nlhs = atoi(argv[1]);
	int nrhs = argc-2;
	mxArray* lhs[nlhs];
	const mxArray* rhs[nrhs];
	for (int i = 0; i < nrhs; i++) {
		rhs[i] = mxCreateString(argv[i+2]);
	}

	_mexFunction(nlhs, lhs, nrhs, rhs);

	cout << "Done.\n";

	return 0;
}
