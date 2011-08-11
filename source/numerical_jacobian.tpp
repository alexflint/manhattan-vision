#include "common_types.h"

#include <boost/function.hpp>

namespace indoor_context {
	// Compute numerical jacobians from a function references
	template <int XD, int YD>
	Matrix<YD,XD> NumericalJacobian(boost::function<Vector<YD>(const Vector<XD>&)> f,
																	const Vector<XD>& x0,
																	const Vector<XD>& h) {
		Vector<YD> y0 = f(x0);
		int xd = x0.size();
		int yd = y0.size();  // note that M might -1; this gets the actual size

		// this multiplication is meaningless; it's just to get the right size matrix
		Matrix<YD,XD> J = y0.as_col() * x0.as_row();
		for (int i = 0; i < xd; i++) {
			Vector<XD> xa = x0, xb = x0;
			xa[i] -= h[i];
			xb[i] += h[i];
			Vector<YD> ya = f(xa);
			Vector<YD> yb = f(xb);
			J.T()[i] = (yb - ya) / (2. * h[i]);
		}
		return J;
	}

	template <int XD, int YD>
	Matrix<YD,XD> NumericalJacobian(boost::function<Vector<YD>(const Vector<XD>&)> f,
																	const Vector<XD>& x0,
																	double delta) {
		Vector<XD> h = x0; // assign x0 only to initialize it to the same size
		h = Ones * delta;
		return NumericalJacobian(f, x0, h);
	}

	template <int XD>
	Vector<XD> NumericalJacobian(boost::function<double(const Vector<XD>&)> f,
															 const Vector<XD>& x0,
															 const Vector<XD>& h) {
		// Wrap f with a vector-valued function
		boost::function<Vector<1>(const Vector<XD>&)> g =
			boost::bind(makeVector<double>, boost::bind(f, _1));
	
		// Get jacobian
		Matrix<1,XD> J = NumericalJacobian(g, x0, h);
		return J[0];
	}

	template <int XD>
	Vector<XD> NumericalJacobian(boost::function<double(const Vector<XD>&)> f,
															 const Vector<XD>& x0,
															 double delta) {
		// Wrap f with a vector-valued function
		boost::function<Vector<1>(const Vector<XD>&)> g =
			boost::bind(makeVector<double>, boost::bind(f, _1));
	
		// Get jacobian
		Matrix<1,XD> J = NumericalJacobian(g, x0, delta);
		return J[0];
	}
}
