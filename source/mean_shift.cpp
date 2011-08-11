#include "common_types.h"
#include "mean_shift.h"

#include "range_utils.tpp"

namespace indoor_context {
	void MeanShift1D::Compute(double window) {
		double kConvergeTol = 1e-6;
		CHECK(!xs.empty());

		vector<double> cur_xs, next_xs;
		copy_all_into(xs, cur_xs);
		sort_all(cur_xs);

		double converged;
		int iters = 0;
		do {
			next_xs.clear();
			converged = true;
			next_xs.clear();
			int start_j = 0;
			for (int i = 0; i < cur_xs.size(); i++) {
				double wa = cur_xs[i]-3*window;
				double wb = cur_xs[i]+3*window;
				double sum = 0, denom = 0;
				while (cur_xs[start_j] < wa) {
					start_j++;
				}
				for (int j = start_j; j < cur_xs.size() && cur_xs[j] < wb; j++) {
					double k = EvalKernel(cur_xs[i]-cur_xs[j],window);
					//SPACED_INDENT; DREPORT(cur_xs[i],cur_xs[j], window, k);
					sum += cur_xs[j]*k;
					denom += k;
				}
				double next_x = sum/denom;
				double change = abs(cur_xs[i]-next_x);
				if (change > kConvergeTol) {
					converged = false;
				}
				next_xs.push_back(next_x);
			}
			swap(next_xs, cur_xs);
			iters++;
 		} while (!converged);

		modes.push_back(cur_xs[0]);
		for (int i = 1; i < cur_xs.size(); i++) {
			if (abs(cur_xs[i]-cur_xs[i-1]) > 10*kConvergeTol) {
				modes.push_back(cur_xs[i]);
			}
		}
		DLOG << "Mean shift converged after " << iters << " iterations";
	}

	double MeanShift1D::EvalKernel(double x, double s) {
		static double k = 1.0 / sqrt(2*M_PI);
		return exp(-x*x/(2.0*s*s)) * k/s;
	}
}

