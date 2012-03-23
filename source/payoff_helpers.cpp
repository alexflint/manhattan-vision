#include "payoff_helpers.h"

#include "common_types.h"
#include "protobuf_utils.h"

#include "io_utils.tpp"
#include "counted_foreach.tpp"

namespace indoor_context {
	void PackMatrix(const MatF& in, proto::MatF& out) {
		out.set_rows(in.Rows());
		out.set_cols(in.Cols());
		int nbytes = in.Rows() * in.Cols() * sizeof(float);
		out.set_data(reinterpret_cast<const void*>(in.DataBlock()), nbytes);
	}

	void PackMatrix(const MatI& in, proto::MatI& out) {
		out.set_rows(in.Rows());
		out.set_cols(in.Cols());
		int nbytes = in.Rows() * in.Cols() * sizeof(int);
		out.set_data(reinterpret_cast<const void*>(in.DataBlock()), nbytes);
	}

	void UnpackMatrix(const proto::MatF& in, MatF& out) {
		int nnbytes = in.rows() * in.cols() * sizeof(float);
		CHECK_EQ(in.data().size(), nnbytes);
		out.Resize(in.rows(), in.cols());
		out.CopyIn(reinterpret_cast<const float*>(in.data().data()));
	}

	void UnpackMatrix(const proto::MatI& in, MatI& out) {
		int nnbytes = in.rows() * in.cols() * sizeof(int);
		CHECK_EQ(in.data().size(), nnbytes);
		out.Resize(in.rows(), in.cols());
		out.CopyIn(reinterpret_cast<const int*>(in.data().data()));
	}

	void PackPayoffs(const DPPayoffs& payoffs,
									 const string& description,
									 proto::PayoffFeature& data) {
		PackMatrix(payoffs.wall_scores[0], *data.mutable_left());
		PackMatrix(payoffs.wall_scores[1], *data.mutable_right());
		if (!description.empty()) {
			data.set_description(description);
		}
	}

	void PackPayoffs(const MatF& payoffs,
									 const string& description,
									 proto::PayoffFeature& data) {
		PackMatrix(payoffs, *data.mutable_left());
		if (!description.empty()) {
			data.set_description(description);
		}
	}

	void UnpackPayoffs(const proto::PayoffFeature& data,
										 DPPayoffs& payoffs) {
		UnpackMatrix(data.left(), payoffs.wall_scores[0]);
		if (data.has_right()) {
			UnpackMatrix(data.right(), payoffs.wall_scores[1]);
		} else {
			payoffs.wall_scores[1] = payoffs.wall_scores[0];
		}
	}

	void PackFeatures(const PayoffFeatures& fset,
										proto::PayoffFeatureSet& data) {
		data.clear_features();
		CHECK_EQ(fset.features.size(), fset.descriptions.size());
		COUNTED_FOREACH(int i, const DPPayoffs& ftr, fset.features) {
			PackPayoffs(ftr, fset.descriptions[i], *data.add_features());
		}
	}

	void UnpackFeatures(const proto::PayoffFeatureSet& data,
											PayoffFeatures& fset) {
		fset.features.resize(data.features_size());
		fset.descriptions.resize(data.features_size());
		for (int i = 0; i < data.features_size(); i++) {
			UnpackPayoffs(data.features(i), fset.features[i]);
			if (data.features(i).has_description()) {
				fset.descriptions[i] = data.features(i).description();
			}
		}
	}

	void WriteFeatures(const string& path, const PayoffFeatures& fset) {
		proto::PayoffFeatureSet p;
		PackFeatures(fset, p);
		WriteProto(path, p);
	}
		
	void ReadFeatures(const string& path, PayoffFeatures& fset) {
		proto::PayoffFeatureSet p;
		ReadLargeProto(path, p);
		UnpackFeatures(p, fset);
	}

	ManhattanHyperParameters::ManhattanHyperParameters(const VecF& w,
																										 float c,
																										 float o)
		: weights(w), corner_penalty(c), occlusion_penalty(o) {
	}

	void PayoffFeatures::Clear() {
		features.clear();
		descriptions.clear();
	}

	void PayoffFeatures::AddCopy(const DPPayoffs& x, const string& description) {
		CHECK_EQ(features.size(), descriptions.size());
		features.push_back(new DPPayoffs);
		x.CopyTo(features.back());
		descriptions.push_back(description);
	}

	void PayoffFeatures::AddCopy(const MatF& x, const string& description) {
		CHECK_EQ(features.size(), descriptions.size());
		features.push_back(new DPPayoffs);
		features.back().wall_scores[0] = x;
		features.back().wall_scores[1] = x;
		descriptions.push_back(description);
	}

	void PayoffFeatures::Compile(const ManhattanHyperParameters& params,
															 DPPayoffs& payoffs) const {
		// Copy penalties
		payoffs.wall_penalty = params.corner_penalty;
		payoffs.occl_penalty = params.occlusion_penalty;

		// Sum features
		CHECK(!features.empty());
		CHECK_EQ(params.weights.Size(), features.size());
		payoffs.Resize(matrix_size(features[0]));
		for (int i = 0; i < params.weights.size(); i++) {
			payoffs.Add(features[i], params.weights[i]);
		}
	}		
}
