// Find optimal indoor Manhattan structures by dynamic programming

#include <SVD.h>
#include <iostream>
#include <VNL/Algo/svd.h>

using namespace TooN;
using namespace std;

void testVNL() {
	VNL::Matrix<double> m(3,2);
	m[0][0] = 0.259724; m[0][1] = -0.254221;
	m[1][0] = -0.224489; m[1][1] = -0.126041;
	m[2][0] = 1; m[2][1] =  -1;
	VNL::Vector<double> y(-0.0189444, -0.990712, -0.134654);
	VNL::Vector<double> x = VNL::SVD<double>(m).Solve(y);
	cout << "VNL answer: " << x << endl;
}

void testToon() {
	Matrix<3,2> m = Zeros;
	m[0] = makeVector(0.259724, -0.254221);
	m[1] = makeVector(-0.224489, -0.126041);
	m[2] = makeVector(1, -1);
	Vector<3> y = makeVector(-0.0189444, -0.990712, -0.134654);
	Vector<2> x = SVD<3,2>(m).backsub(y);
	cout << "Toon answer: " << x << endl;
}

int main(int argc, char **argv) {
	testToon();
	testVNL();
	return 0;
}
