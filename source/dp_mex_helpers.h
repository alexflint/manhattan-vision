/*
 * dp_mex_helpers.h
 *
 *  Created on: 12 Aug 2010
 *      Author: alexf
 */
#pragma once

#include <boost/format.hpp>

#include "common_types.h"
#include "matlab_utils.h"
#include "map.h"
#include "map.pb.h"

namespace indoor_context {
using namespace toon;

template <typename T>
string sformat(const char* s, const T& x) {
	return str(format(s) % x);
}
template <typename T1, typename T2>
string sformat(const char* s, const T1& x1, const T2& x2) {
	return str(format(s) % x1 % x2);
}
template <typename T1, typename T2, typename T3>
string sformat(const char* s, const T1& x1, const T2& x2, const T3& x3) {
	return str(format(s) % x1 % x2 % x3);
}
template <typename T1, typename T2, typename T3, typename T4>
string sformat(const char* s,
               const T1& x1,
               const T2& x2,
               const T3& x3,
               const T4& x4) {
	return str(format(s) % x1 % x2 % x3 % x4);
}



KeyFrame& LoadFrameByCaseName(const string& case_name, Map& map, proto::TruthedMap& gt_map) {
	int colon_pos = case_name.find(':');
	if (colon_pos == string::npos) {
		mexErrMsgTxt("Malformed case name");
	}
	string sequence_name = case_name.substr(0, colon_pos);
	int frame_index = lexical_cast<int>(case_name.substr(colon_pos+1));
	DREPORT(sequence_name, frame_index);

	// Make the mape file
	format map_file_tpl("/homes/50/alexf/data/sequences/%s/ground_truth/truthed_map.pro");
	string map_file = str(map_file_tpl % sequence_name);
	DREPORT(map_file);

	// Load the map
	map.LoadWithGroundTruth(map_file, gt_map);
	CHECK_INDEX(frame_index, map.kfs);
	return map.kfs[frame_index];
}


class CompatibilityFeatures {
public:
	static const int kFeatureLength = 6;
	typedef toon::Vector<kFeatureLength> Feature;

	GuidedLineDetector line_detector;
	IsctGeomLabeller line_sweeper;

	Vector<3*kFeatureLength> compat;

	void Compute(const PosedImage& pim,
	             const MatI& labels) {
		CHECK_EQ(labels.Rows(), pim.ny());
		CHECK_EQ(labels.Cols(), pim.nx());

		// Compute line sweeps
		line_detector.Compute(pim);
		line_sweeper.Compute(pim, line_detector.detections);

		//  Sum the feature vectors
		compat = Zeros;
		for (int y = 0; y < pim.ny(); y++) {
			const int* labelrow = labels[y];
			const PixelRGB<byte>* imrow = pim.rgb[y];
			const int* sweeprow = line_sweeper.orient_map[y];
			for (int x = 0; x < pim.nx(); x++) {
				Vector<kFeatureLength> ftr = Zeros;
				ftr[0] = imrow[x].r;
				ftr[1] = imrow[x].g;
				ftr[2] = imrow[x].b;
				ftr[3] = sweeprow[x] == 0 ? 1.0 : 0.0;
				ftr[4] = sweeprow[x] == 1 ? 1.0 : 0.0;
				ftr[5] = sweeprow[x] == 2 ? 1.0 : 0.0;
				CHECK_INTERVAL(labelrow[x], 0, 2) << format("x=%d, y=%d") % x % y;
				compat.slice(labelrow[x]*6, 6) += ftr;
			}
		}
	}
};
}
