#include "common_types.h"
#include "gaussian.h"

namespace indoor_context {
	// Performs mean-shift clustering
	class MeanShift1D {
	public:
		vector<double> xs;
		vector<double> modes;
		void Compute(double window);
		static double EvalKernel(double x, double s);
	};
}
