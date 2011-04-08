#include "entrypoint_types.h"

#include "numerical_jacobian.tpp"

Vector<4> linear(const Vec3& x) {
	Matrix<4,3> m = Zeros;
	m[0] = makeVector(-5, 0, 1.1);
	m[1] = makeVector(1e-2, 1e+2, 17);
	m[2] = makeVector(0, 0, -12.3);
	Vector<4> y = m*x;
	y[3] = x[0]*x[0] + 3*x[0] + 100;
	return y;
}

double quad(const Vec3& x) {
	return x[0]*x[0] + 3*x[0]  +  x[1]*x[1]*x[1];
}

int main(int argc, char **argv) {
	boost::function<double(const Vec3&)> f = &quad;
	Vec3 x0 = -1 * Ones;
	Vec3 J = NumericalJacobian(f, x0, 1e-3);
	DREPORT(J);
	return 0;
}
