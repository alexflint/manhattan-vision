
#include <VNL/vectorref.h>
#include <VNL/vectorfixedref.h>
#include <VNL/Algo/symmetriceigensystem.h>

#include "common_types_vw.h"

typedef VectorFixed<4,double> QtrnD;
typedef VectorFixed<4,float> QtrnF;

// Computes the product of two quaternions
template <typename T>
VectorFixed<4,T> QtrnProduct(const VectorFixed<4,T>& q,
		const VectorFixed<4,T>& r) {
	return VectorFixed<4,T>(q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3],
			q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2],
			q[0]*r[2] + q[2]*r[0] + q[3]*r[1] - q[1]*r[3],
			q[0]*r[3] + q[3]*r[0] + q[1]*r[2] - q[2]*r[1]);
}

// Computes the product of two quaternions (old implementation, used
// for verification)
template <typename T>
VectorFixed<4,T> QtrnProductSlow(const VectorFixed<4,T>& a,
		const VectorFixed<4,T>& b) {
	const T s = a[0];
	const T t = b[0];
	// VectorFixedRef lacks a const constructor, so use const_cast
	const VectorFixedRef<3,T> v(const_cast<double*>(&a[1]));
	const VectorFixedRef<3,T> w(const_cast<double*>(&b[1]));
	VectorFixed<4,T> out;
	out[0] = s*t - DotProduct(v, w);  // real part
	VectorFixedRef<3,T> imag(&out[1]);
	imag = s*w + t*v + Cross3D(v, w);  // imag part
	return out;
}

// Computes the inverse of a quaternion
template <typename T>
VectorFixed<4,T> QtrnInverse(const VectorFixed<4,T>& a) {
	const T len = a.SquaredMagnitude();
	return VectorFixed<4,T>(a[0]/len, -a[1]/len, -a[2]/len, -a[3]/len);
}

// Computes q^-1 * v * q
template <typename T>
VectorFixed<4,T> QtrnConjugate(const VectorFixed<4,T>& v,
		const VectorFixed<4,T>& q) {
	return QtrnProduct(QtrnProduct(q, v), QtrnInverse(q));
}

// Transform a vector x by the rotation specified by a quaternion q
template <typename T>
VectorFixed<3,T> QtrnTransform(const VectorFixed<3,T>& x,
		const VectorFixed<4,T>& q) {
	const VectorFixed<4,T> v(0, x[0], x[1], x[2]);
	const VectorFixed<4,T> res = QtrnConjugate(v, q);
	return VectorFixed<3,T>(res[1], res[2], res[3]);
}

// Construct a quaternion representing a rotation around axis of angle theta.
template <typename T>
VectorFixed<4,T> BuildRotationQtrn(const VectorFixed<3,T>& axis,
		const T& theta) {
	// Here we really do want Magnitude (not SquaredMagnitude) because
	// we are trying to create a unit vector
	const T len = axis.Magnitude();
	const T cth = cos(theta/2.0);
	const T sth = sin(theta/2.0);
	return QtrnD(cth, sth*axis[0]/len, sth*axis[1]/len, sth*axis[2]/len);
}

// Fit a quaternion to an arbitrary matrix
template <typename T>
VectorFixed<4,T> FitQtrn(const MatrixFixed<3,3,T>& m, double& out_eval) {
	// Create the estimation matrix
	MatrixFixed<4,4,T> K;
	K[0][0] = m[0][0] - m[1][1] - m[2][2];
	K[0][1] = m[1][0] + m[0][1];
	K[0][2] = m[2][0] + m[0][2];
	K[0][3] = m[1][2] - m[2][1];
	K[1][0] = m[1][0] + m[0][1];
	K[1][1] = m[1][1] - m[0][0] - m[2][2]; 
	K[1][2] = m[2][1] + m[1][2];
	K[1][3] = m[2][0] - m[0][2];
	K[2][0] = m[2][0] + m[0][2];
	K[2][1] = m[2][1] + m[1][2];
	K[2][2] = m[2][2] - m[0][0] - m[1][1]; 
	K[2][3] = m[0][1] - m[1][0];
	K[3][0] = m[1][2] - m[2][1];
	K[3][1] = m[2][0] - m[0][2];
	K[3][2] = m[0][1] - m[1][0];
	K[3][3] = m[0][0] + m[1][1] + m[2][2];
	K /= 3.0;
	SymmetricEigensystem<T> eig(K);
	out_eval = eig.GetEigenvalue(3);
	VectorFixed<4,T> v = eig.GetEigenvector(3);
	// Must rotate entries due to our quaternion convention
	// The last three elems seem to need negation, not sure why
	return VectorFixed<4,T>(v[3], v[0], v[1], v[2]);
}

// Fit a quaternion to an arbitrary matrix
template <typename T>
VectorFixed<4,T> FitQtrn(const MatrixFixed<3,3,T>& m) {
	double eval;
	return FitQtrn(m, eval);
}

// Fit a quaternion to an aribtrary coordinate transform. The three
// vectors represent the X, Y, and Z axes under the transformed
// coordinate system. The last parameter is the largest eigenvalue
// (the returned quaternion is the corresponding eigenvector)
template <typename T>
VectorFixed<4,T> FitQtrn(const VectorFixed<3,T>& ex,
		const VectorFixed<3,T>& ey,
		const VectorFixed<3,T>& ez,
		double& out_eval) {
	MatrixFixed<3,3,T> m;
	m.SetRow(0, ex);
	m.SetRow(1, ey);
	m.SetRow(2, ez);
	return FitQtrn(m, out_eval);
}

// Fit a quaternion to an aribtrary coordinate transform. The three
// vectors represent the X, Y, and Z axes under the transformed
// coordinate system.
template <typename T>
VectorFixed<4,T> FitQtrn(const VectorFixed<3,T>& ex,
		const VectorFixed<3,T>& ey,
		const VectorFixed<3,T>& ez) {
	double eval;
	return FitQtrn(ex, ey, ez, eval);
}

// Convert a quaternion to a 3x3 rotation matrix
template <typename T>
MatrixFixed<3,3,T> QtrnToRotation(const VectorFixed<4,T>& q) {
	const T w = q[0];
	const T x = q[1];
	const T y = q[2];
	const T z = q[3];
	const T len = q.SquaredMagnitude();
	const T s = (len > 1e-8 ? 2.0/len : 0.0);
	const T X = x*s, Y = y*s, Z = z*s;
	const T wX = w*X, wY = w*Y, wZ = w*Z;
	const T xX = x*X, xY = x*Y, xZ = x*Z;
	const T yY = y*Y, yZ = y*Z, zZ = z*Z;
	MatrixFixed<3,3,T> m;
	m[0][0] = 1.0-(yY+zZ);
	m[0][1] = xY-wZ;
	m[0][2] = xZ+wY;
	m[1][0] = xY+wZ;
	m[1][1] = 1.0-(xX+zZ);
	m[1][2] = yZ-wX;
	m[2][0] = xZ-wY;
	m[2][1] = yZ+wX;
	m[2][2] = 1.0-(xX+yY);
	return m;
}

int main(int argc, char **argv) {
	Vec3D ex(0, 1, 0);
	Vec3D ey(0, 0, 1);
	Vec3D ez(1, 0, 0);

	QtrnD est = FitQtrn(ex, ey, ez);
	DREPORT(est);
	DREPORT(QtrnTransform(Vec3D(1,0,0), est));

	return 0;
}
