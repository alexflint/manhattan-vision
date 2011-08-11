#include <VNL/Algo/symmetriceigensystem.h>

#include <TooN/SymEigen.h>

#include "common_types.h"
#include "quaternion.h"

// Test case for conversion routines:
// Quaternion: 
//   -0.0365465 0.759656 -0.0515363 0.647249
// Rotation matrix:
//   0.156826 -0.0309903 0.98714
//   -0.125609 -0.992017 -0.0111881
//   0.979606 -0.122239 -0.159466

namespace indoor_context {
	// Computes the product of two quaternions
	template <typename T>
	toon::Vector<4,T> QtrnProduct(const toon::Vector<4,T>& q,
																const toon::Vector<4,T>& r) {
		return makeVector(q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3],
											q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2],
											q[0]*r[2] + q[2]*r[0] + q[3]*r[1] - q[1]*r[3],
											q[0]*r[3] + q[3]*r[0] + q[1]*r[2] - q[2]*r[1]);
	}

	// Computes the product of two quaternions (old implementation, used
	// for verification)
	template <typename T>
	toon::Vector<4,T> QtrnProductSlow(const toon::Vector<4,T>& a,
																		const toon::Vector<4,T>& b) {
		const toon::Vector<3,T> v = a.template slice<1,3>();
		const toon::Vector<3,T> w = b.template slice<1,3>();
		toon::Vector<4,T> out;
		out[0] = a[0]*a[1] - v*w;  // real part
		out.template slice<1,3>() = a[0]*w + a[1]*v + (v^w);  // imag part
		return out;
	}

	// Computes the multiplicative inverse of a quaternion
	template <typename T>
	toon::Vector<4,T> QtrnInverse(const toon::Vector<4,T>& a) {
		const T len = norm_sq(a);
		return makeVector(a[0]/len, -a[1]/len, -a[2]/len, -a[3]/len);
	}

	// Computes q * v * q^-1
	template <typename T>
	toon::Vector<4,T> QtrnConjugate(const toon::Vector<4,T>& v,
																	const toon::Vector<4,T>& q) {
		return QtrnProduct(QtrnProduct(q, v), QtrnInverse(q));
	}

	// Transforms a vector x by a quaternion q
	template <typename T>
	toon::Vector<3,T> QtrnTransform(const toon::Vector<3,T>& x,
																	const toon::Vector<4,T>& q) {
		const toon::Vector<4,T> v = makeVector(0.0, x[0], x[1], x[2]);
		const toon::Vector<4,T> res = QtrnConjugate(v, q);
		return makeVector(res[1], res[2], res[3]);
	}

	// Constructs a quaternion representing a rotation around the
	// specified axis of angle theta
	template <typename T>
	toon::Vector<4,T> BuildRotationQtrn(const toon::Vector<3,T>& axis,
																			const T& theta) {
		// Here we really do want Magnitude (not SquaredMagnitude) because
		// we are trying to create a unit vector
		const T m = norm(axis);
		const T c = cos(theta/2.0);
		const T s = sin(theta/2.0);
		return QtrnD(c, s*axis[0]/m, s*axis[1]/m, s*axis[2]/m);
	}

	// Convert a quaternion to a 3x3 rotation matrix
	template <typename T>
	toon::Matrix<3,3,T> QtrnToRotation(const toon::Vector<4,T>& q) {
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
		toon::Matrix<3,3,T> m;
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

	// Fit a quaternion to an arbitrary matrix
	template <typename T>
	toon::Vector<4,T> FitQtrn(const toon::Matrix<3,3,T>& m, double& out_eig_value) {
		// Create the estimation matrix
		toon::Matrix<4,4,T> K;
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
		toon::SymEigen<4,T> eig(K);
		out_eig_value = eig.get_evalues()[3];
		toon::Vector<4,T> v = eig.get_evectors()[3];
		// Must rotate entries due to our quaternion convention
		// Apparently the first element needs negation
		return makeVector(-v[3], v[0], v[1], v[2]);
	}

	// Fit a quaternion to an arbitrary matrix
	template <typename T>
	toon::Vector<4,T> FitQtrn(const toon::Matrix<3,3,T>& m) {
		double eig_value;
		return FitQtrn(m, eig_value);
	}

	// Fit a quaternion to an aribtrary coordinate transform. The three
	// vectors represent the X, Y, and Z axes under the transformed
	// coordinate system. The last parameter is the largest eigenvalue
	// (the returned quaternion is the corresponding eigenvector)
	template <typename T>
	toon::Vector<4,T> FitQtrn(const toon::Vector<3,T>& ex,
														const toon::Vector<3,T>& ey,
														const toon::Vector<3,T>& ez,
														double& out_eig_value) {
		toon::Matrix<3,3,T> m;
		m.SetRow(0, ex);
		m.SetRow(1, ey);
		m.SetRow(2, ez);
		return FitQtrn(m, out_eig_value);
	}

	// Fit a quaternion to an aribtrary coordinate transform. The three
	// vectors represent the X, Y, and Z axes under the transformed
	// coordinate system.
	template <typename T>
	toon::Vector<4,T> FitQtrn(const toon::Vector<3,T>& ex,
														const toon::Vector<3,T>& ey,
														const toon::Vector<3,T>& ez) {
		double eval;
		return FitQtrn(ex, ey, ez, eval);
	}
}
