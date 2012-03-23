#include "entrypoint_types.h"
#include "payoff_helpers.h"
#include "map.h"
#include "map_io.h"
#include "building_features.h"
#include "monocular_payoffs.h"
#include "protobuf_utils.h"
#include "payoffs.pb.h"
#include "timer.h"

#include "format_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);

	Map map;
	proto::TruthedMap gt_map;
	LoadXmlMapWithGroundTruth(GetMapPath("lab_kitchen1"), map, gt_map);
	const proto::FloorPlan& fp = gt_map.floorplan();

	Frame& frame = map.frames[25];
	frame.LoadImage();

	DPGeometryWithScale geometry(frame.image.pc(), fp.zfloor(), fp.zceil());

	PhotometricFeatures ftr_gen("all");
	FeaturePayoffGen payoff_gen;
	PayoffFeatures fset;

	// Compute fset
	TIMED ("Compute features")
		ftr_gen.Compute(frame.image);

	payoff_gen.Compute(*ftr_gen.features[0], geometry);
	const MatF& m0 = payoff_gen.hpayoffs.wall_scores[0];

	// Convert each to payoffs
	TIMED ("Copy into featureset")
	for (int i = 0; i < ftr_gen.features.size(); i++) {
		payoff_gen.Compute(*ftr_gen.features[i], geometry);
		const string& s = ftr_gen.feature_strings[i];
		fset.AddCopy(payoff_gen.vpayoffs[0], fmt("%s [left]", s));
		fset.AddCopy(payoff_gen.vpayoffs[1], fmt("%s [right]", s));
		fset.AddCopy(payoff_gen.hpayoffs, fmt("%s [floor/ceil]", s));
	}

	// Write to file
	proto::PayoffFeatureSet p;
	TIMED ("Pack features")
		PackFeatures(fset, p);
	TIMED ("Write features")
		WriteProto("featureset.protodata", p);

	// Read from file
	proto::PayoffFeatureSet q;
	TIMED ("Read features")
		ReadLargeProto("featureset.protodata", q);

	PayoffFeatures gset;
	TIMED ("Unpack features")
		UnpackFeatures(q, gset);

	CHECK(gset.features[0].wall_scores[0] == fset.features[0].wall_scores[0]);

	return 0;
}
