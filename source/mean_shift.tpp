#include "common_types.h"
#include "gaussian.h"

namespace indoor_context {
	// Performs mean-shift clustering
	class MeanShift1D {
	public:
		vector<double> xs;
		vector<double> modes;
		
		void Compute();
		static double EvalKernel(double x);
	};

	void MeanShift1D::Compute(double window) {
		double kConvergeTol = 1e-6;

		vector<double> next_xs;
		CHECK(!xs.empty());
		double converged;
		int iters = 0;
		do {
			next_xs.clear();
			converged = true;
			for (int i = 0; i < xs.size(); i++) {
				next_xs.clear();
				double sum = 0, denom = 0;
				for (int j = 0; j < xs.size(); j++) {
					double k = EvalKernel( abs(xs[i]-xs[j])/window );
					sum += xs[j]*k;
					denom += k;
				}
				double next_x = sum/denom;
				double change = (x-next_x);
				if (change > kConvergeTol) {
					converged = false;
				}
				next_xs.push_back(next_x);
			}
			swap(next_xs, xs);
			iters++;
		}
		DLOG << "Mean shift converged after " << iters << " iterations";
	}
}

