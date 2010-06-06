#include "misc.h"

#include <string>

namespace indoor_context {

bool StringContains(const string& haystack, const string& needle) {
	return haystack.find(needle) != string::npos;
}

	/*istream& operator>>(istream& s, vector<VecD>& vs) {
		int dim;
		s >> dim;
		VecD v(dim);
		while (s >> v) {
			vs.push_back(v);
		}
		return s;
	}

	ostream& operator<<(ostream& s, const vector<VecD>& vs) {
		int dim = vs.size() > 0 ? vs[0].Size() : 0;
		s << dim << "\n\n";
		copy(vs.begin(), vs.end(), ostream_iterator<VecD>(s, "\n\n"));
	}


	int Sample(const vector<float> cum_distr) {
		float r = rand() * cum_distr.back() / RAND_MAX;
		vector<float>::const_iterator pos = upper_bound(cum_distr.begin(), cum_distr.end(), r);
		return pos - cum_distr.begin();
	}

	void Sample(const Table<3,int>& distr, int& i, int& j, int& k) {
		// Generate cumulative distribution
		float sum = 0;
		vector<float> cum_distr;
		const int* data = distr.begin();
		for (int a = 0; a < distr.size(); a++) {
			sum += data[a];
			cum_distr.push_back(sum);
		}

		// Sample and retrieve indices
		int index = Sample(cum_distr);
		VectorFixed<3, int> dims = distr.dimensions();
		k = index % dims[2];
		index /= dims[2];
		j = index % dims[1];
		index /= dims[1];
		i = index;
	}*/
}
