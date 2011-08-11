
#include "kmeans.h"

#include <ext/algorithm>
#include <vector>

#include <boost/foreach.hpp>

#include <VNL/matrix.h>
#include <VNL/vector.h>
#include <VNL/sample.h>

#include "common_types.h"

namespace indoor_context {

bool KMeans::Estimate(const vector<VecD>& pts,
                      int ncomps,
                      vector<VecD>& means,
                      VecI& parents) {
	assert(means.size() == 0);
	assert(parents.Size() == pts.size());
	assert(pts.size() >= ncomps);

	// Parameters
	gvar3<int> gvMaxIterations("KMeans.MaxIterations");
	gvar3<double> gvExitThreshold("KMeans.ExitThreshold");

	// Initialize the means by randomly sampling from the input points
	// without repetition
	random_sample_n(pts.begin(), pts.end(), back_inserter(means), ncomps);

	// Initialize the newmeans vector
	VecI childcounts(ncomps);
	vector<VecD> newmeans(ncomps, VecD(pts[0].size()));

	// Start iterating
	for (int i = 0; i < *gvMaxIterations; i++) {
		// Re-initialize
		childcounts.Fill(0);
		BOOST_FOREACH(VecD& v, newmeans) {
			v.Fill(0.0);
		}

		// Compute assignments and sum new means
		for (int j = 0; j < pts.size(); j++) {
			double mindist = INFINITY;
			int parent;
			for (int k = 0; k < ncomps; k++) {
				double dist = VectorSSD(pts[j], means[k]);
				if (dist < mindist) {
					mindist = dist;
					parent = k;
				}
			}
			newmeans[parent] += pts[j];
			childcounts[parent]++;
			parents[j] = parent;
		}

		// Normalize new means and compare with old means
		double maxchange = 0.0;
		for (int k = 0; k < means.size(); k++) {
			if (childcounts[k] > 0) {
				newmeans[k] /= childcounts[k];
			} else {
				DLOG << "Warning: K-Means component " << k << " has no support" << endl;
			}
			double change = VectorSSD(means[k], newmeans[k]);
			if (change > maxchange) {
				maxchange = change;
			}
		}

		// Determine whether to stop now
		if (maxchange < *gvExitThreshold) {
			DLOG << "K-means converged after " << i << " iterations" << endl;
			return true;
		}

		// Swap old for new
		swap(means, newmeans);
	}

	DLOG << "K-means failed to converge after "
			<< *gvMaxIterations << " iterations" << endl;
	return false;
}

bool KMeans::Estimate(const vector<VecD>& pts,
                      int ncomps,
                      vector<VecD>& means,
                      MatD& resps) {
	assert(resps.Rows() == pts.size());
	assert(resps.Cols() == ncomps);

	VecI parents(pts.size());
	const bool ret = Estimate(pts, ncomps, means, parents);

	resps.Fill(0.0);
	for (int j = 0; j < pts.size(); j++) {
		resps[j][parents[j]] = 1.0;
	}

	return ret;
}

}
