#pragma once

#include "common_types.h"

namespace indoor_context {
	class KMeans {
	public:
		// Run K-means, write the final cluster centres to OUT_MEANS and
		// write the parent cluster for each input point in OUT_PARENTS
		static bool Estimate(const vector<VecD>& pts,
												 int ncomps,
												 vector<VecD>& out_means,
												 VecI& out_parents);

		// Run K-means, write the final cluster centres to OUT_MEANS and
		// write the final responsibility matrix to
		// OUT_RESPONSIBILITIES. Each row will consist of a 1 at the column
		// corresponding to the parent cluster for that point and zeros
		// elsewhere.
		static bool Estimate(const vector<VecD>& pts,
												 int ncomps,
												 vector<VecD>& out_means,
												 MatD& out_responsibilities);
	};
}
