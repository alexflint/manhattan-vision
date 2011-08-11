#include <vector>

#include <so3.h>

#include "common_types.h"
#include "misc.h"
#include "rotation_estimator.h"

using namespace TooN;
using namespace indoor_context;

lazyvar<double> gvAbsTol("RotationEstimator.AbsTol");
lazyvar<double> gvRelTol("RotationEstimator.RelTol");

int main(int argc, char **argv) {
	indoor_context::InitVars(argc, argv);

	int n;
	vector<Vector<3> > lines;
	vector<int> labels;
	
	// Input
	cin >> n;
	for (int i = 0; i < n; i++) {
		int label;
		Vector<3> v;
		cin >> label;
		cin >> v;
		lines.push_back(v);
		labels.push_back(label);
		cout << v << "   " << label << endl;
	}

	Matrix<> resps(lines.size(), 3);
	resps = TooN::Zeros;
	for (int i = 0; i < n; i++) {
		resps[i][labels[i]] = 1;
	}

	SO3<> init;
	RotationEstimator est;
	est.Compute(lines, resps, init);

	return 0;
}
