#pragma once

#include "common_types.h"

namespace indoor_context {

	// Fast eigen decomposition routines for 2x2 matrices.
	// For sensible results T should be double or float
	template <typename T>
	class EigenSystem2D {
	public:
		double eigval_large, eigval_small;  // The two Eigenvalues
		toon::Vector<2,T> eigvec_large, eigvec_small;  // The two Eigenvectors

		// Compute the decomposition
		EigenSystem2D(const toon::Matrix<2,2,T>& A) {
			Compute(A, eigval_large, eigval_small, eigvec_large, eigvec_small);
		}

		// Compute the eigen decomposition for the matrix A
		static void Compute(const toon::Matrix<2,2,T>& A,
												T& eigval_large,
												T& eigval_small,
												toon::Vector<2,T>& eigvec_large,
												toon::Vector<2,T>& eigvec_small) {
			// Unpack the matrix
			const T& a = A[0][0];
			const T& b = A[0][1];
			const T& c = A[1][0];
			const T& d = A[1][1];
		
			// Compute eigvalues
			const T trace = a+d;
			const T det = a*d - b*c;
			const T lambda1 = 0.5 * (trace + sqrt(trace*trace - 4*det));
			const T lambda2 = 0.5 * (trace - sqrt(trace*trace - 4*det));
			eigval_large = max(lambda1, lambda2);
			eigval_small = min(lambda1, lambda2);
		
			// Compute eigvectors
			ComputeEigenvector(A, eigval_large, eigvec_large);
			ComputeEigenvector(A, eigval_small, eigvec_small);
		}

		// Compute the eigvector corresponding to the given eigvalue
		static void ComputeEigenvector(const toon::Matrix<2,2,T>& A,
																	 const T& eigval,
																	 toon::Vector<2,T>& eigvec) {
			const T a = A[0][0] - eigval;
			const T b = A[0][1];
			const T c = A[1][0];
			const T d = A[1][1] - eigval;
			T r, s, norm;
			if (max(abs(a),abs(b)) > max(abs(c),abs(d))) {
				r = a;
				s = b;
			} else {
				r = c;
				s = d;
			}
			if (abs(r) > abs(s)) {
				eigvec[1] = 1;
				eigvec[0] = -s/r;
				norm = sqrt(1+(s*s)/(r*r));
			} else {
				eigvec[0] = 1;
				eigvec[1] = -r/s;
				norm = sqrt(1+(r*r)/(s*s));
			}
			eigvec /= norm;
		}
	};

}
