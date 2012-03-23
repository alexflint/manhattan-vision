#include "common_types.h"

#include <boost/function.hpp>

namespace indoor_context {
	// Compute numerical jacobians from a function references
	template <int XD, int YD>
	toon::Matrix<YD,XD> NumericJacobian(
         boost::function<toon::Vector<YD>(const toon::Vector<XD>&)> f,
				 const toon::Vector<XD>& x0,
				 const toon::Vector<XD>& h) {
		toon::Vector<YD> y0 = f(x0);
		int xd = x0.size();
		int yd = y0.size();  // note that M might -1; this gets the actual size

		// this multiplication is meaningless; it's just to get the right size matrix
		toon::Matrix<YD,XD> J = y0.as_col() * x0.as_row();
		for (int i = 0; i < xd; i++) {
			toon::Vector<XD> xa = x0, xb = x0;
			xa[i] -= h[i];
			xb[i] += h[i];
			toon::Vector<YD> ya = f(xa);
			toon::Vector<YD> yb = f(xb);
			J.T()[i] = (yb - ya) / (2. * h[i]);
		}
		return J;
	}

	template <int XD, int YD>
	toon::Matrix<YD,XD> NumericJacobian(boost::function<toon::Vector<YD>(const toon::Vector<XD>&)> f,
																	const toon::Vector<XD>& x0,
																	double delta) {
		toon::Vector<XD> h = x0; // assign x0 only to initialize it to the same size
		h = toon::Ones * delta;
		return NumericJacobian(f, x0, h);
	}

	template <int XD>
	toon::Vector<XD> NumericJacobian(boost::function<double(const toon::Vector<XD>&)> f,
																	 const toon::Vector<XD>& x0,
																	 const toon::Vector<XD>& h) {
		// Wrap f with a vector-valued function
		boost::function<toon::Vector<1>(const toon::Vector<XD>&)> g =
			boost::bind(toon::makeVector<double>, boost::bind(f, _1));
	
		// Get jacobian
		toon::Matrix<1,XD> J = NumericJacobian(g, x0, h);
		return J[0];
	}

	template <int XD>
	toon::Vector<XD> NumericJacobian(boost::function<double(const toon::Vector<XD>&)> f,
																	 const toon::Vector<XD>& x0,
																	 double delta) {
		// Wrap f with a vector-valued function
		boost::function<toon::Vector<1>(const toon::Vector<XD>&)> g =
			boost::bind(toon::makeVector<double>, boost::bind(f, _1));
	
		// Get jacobian
		toon::Matrix<1,XD> J = NumericJacobian(g, x0, delta);
		return J[0];
	}
}
