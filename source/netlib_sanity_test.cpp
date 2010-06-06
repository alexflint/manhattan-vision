// This file computes a small SVD using toon during static
// initialization. If linked to the NetLib sources included within VNL
// then this will fail. To fix this the LD_PRELOAD environment
// variable should be set to the path to liblapack.so.

#include <iostream>
#include <SVD.h>
using namespace std;
using namespace TooN;

class NetLibSanityTest;
class NetLibSanityTest {
public:
	NetLibSanityTest() {
		Matrix<3,2> m;
		m[0] = makeVector(1,0);
		m[1] = makeVector(0,1);
		m[2] = makeVector(1,1);
		Vector<3> y = makeVector(4,1,5);
		Vector<2> x = SVD<>(m).backsub(y);
		Vector<2> correct = makeVector(4,1);
		if (norm_sq(x-correct) > 1e-8) {
			cout << "Error: TooN is linked to the wrong NetLib and SVD computation are producing garbage results (x=" << x << " correct=" << correct << ")\nRun:\n  export LD_PRELOAD=/usr/lib/liblapack.so\nSee details in netlib_sanity_test.cpp" << endl;
			exit(-1);
		}
	}
};

NetLibSanityTest __test;
