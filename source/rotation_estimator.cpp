#include "rotation_estimator.h"
#include "common_types.h"

//#include "numeric_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace TooN;

	lazyvar<double> gvAbsTol("RotationEstimator.AbsTol");
	lazyvar<double> gvRelTol("RotationEstimator.RelTol");
	lazyvar<double> gvMaxSteps("RotationEstimator.MaxSteps");

	template <typename Matrix>
	TooN::Vector<2> size(const Matrix& mat) {
		return TooN::makeVector(mat.num_rows(), mat.num_cols());
	}

	SO3<>& RotationEstimator::Compute(const vector<Vector<3> >& lines,
																		const Matrix<>& responsibilities,
																		const SO3<>& initial_guess) {
		CHECK_EQ(lines.size(), responsibilities.num_rows());

		// Compose the data matrix
		Matrix<> data(lines.size(), 3);
		for (int i = 0; i < lines.size(); i++) {
			data.slice(i,0,1,3) = lines[i].as_row();
		}

		// Descend until convergence
		Reset(initial_guess);
		while (!converged && num_steps < *gvMaxSteps) {
			Step(data, responsibilities);
		}
		DLOG_N << "SO(3) "
					 << (converged ? "converged" : "did not converge")
					 << " after "<<num_steps<<" iters. (Precisions:";
		BOOST_FOREACH(double x, residuals) {
			DLOG_N << " " << -floori(log10(x));
		}
		DLOG << ")\n";
	}

	void RotationEstimator::Reset() {
		SO3<> ident;
		Reset(ident);
	}

	void RotationEstimator::Reset(const SO3<>& R_init) {
		R = R_init;
		num_steps = 0;
		converged = false;
		residual = INFINITY;
		residuals.clear();
	}

	void RotationEstimator::Step(const Matrix<>& X, const Matrix<>& wts) {
		// Compute jacobian
		int n = X.num_rows();
		Jf = Zeros;
		double fprev = residual;
		double f = 0;		
		for (int i = 0; i < 3; i++) {
			Matrix<3> JR_ei;
			for (int j = 0; j < 3; j++) {
				JR_ei.slice(0,j,3,1) = SO3<>::generator_field(j, GetAxis<3>(i)).as_col();
			}

			Vector<3> w_X_R0 = (wts.slice(0,i,n,1).T() * X * R)[0];
			double f_cur = w_X_R0[i];
			if (!isfinite(f_cur)) {
				INDENTED DREPORT(f_cur, i, R, w_X_R0);
			}
			Vector<3> J_cur = 2 * f_cur * w_X_R0 * JR_ei;
			f += f_cur*f_cur;
			Jf += J_cur;
		}

		// TODO: only accept if residual has decreased, otherwise change step size

		// Save current residuals.
		residual = f;
		residuals.push_back(f);

		// Converged?
		if (abs(f) < *gvAbsTol) {// || fprev-f < *gvRelTol*f) {
			converged = true;
			return;
		}

		// Step
		const double step = f / (Jf*Jf);
		R *= SO3<>::exp(-step*Jf);
		num_steps++;
	}

}
