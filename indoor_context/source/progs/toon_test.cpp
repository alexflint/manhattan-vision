#include <iostream>
#include <so3.h>
#include <determinant.h>
using namespace std;
using namespace TooN;
int main(int argc, char **argv) {
	//m[0] = makeVector(-0.00852668, 0.020567, 0.999752);
	//m[1] = makeVector(0.0187881, 0.999615, -0.0204039),
	//m[2] = makeVector(-0.999787, 0.0186094, -0.00890981);
	Matrix<3> m = Identity;
	m[0][0] = -1;
	SO3<> r(m);
	cout << determinant(m) << endl;
	cout << r << endl << SO3<>::exp(r.ln()) << endl;
}
