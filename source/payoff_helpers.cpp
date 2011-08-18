#include "payoff_helpers.h"

#include <boost/filesystem/operations.hpp>

#include "common_types.h"
#include "io_utils.tpp"

namespace indoor_context {
	void PackMatrix(const MatF& in, proto::MatF& out) {
		out.set_rows(in.Rows());
		out.set_cols(in.Cols());
		out.clear_entries();
		for (int y = 0; y < in.Rows(); y++) {
			const float* row = in[y];
			for (int x = 0; x < in.Cols(); x++) {
				out.add_entries(row[x]);
			}
		}
	}

	void PackMatrix(const MatI& in, proto::MatI& out) {
		out.set_rows(in.Rows());
		out.set_cols(in.Cols());
		out.clear_entries();
		for (int y = 0; y < in.Rows(); y++) {
			const int* row = in[y];
			for (int x = 0; x < in.Cols(); x++) {
				out.add_entries(row[x]);
			}
		}
	}

	void UnpackMatrix(const proto::MatF& in, MatF& out) {
		CHECK_EQ(in.entries_size(), in.rows()*in.cols());
		out.Resize(in.rows(), in.cols());
		int i = 0;
		for (int y = 0; y < in.rows(); y++) {
			float* row = out[y];
			for (int x = 0; x < in.cols(); x++) {
				row[x] = in.entries(i++);
			}
		}
	}

	void UnpackMatrix(const proto::MatI& in, MatI& out) {
		CHECK_EQ(in.entries_size(), in.rows()*in.cols());
		out.Resize(in.rows(), in.cols());
		int i = 0;
		for (int y = 0; y < in.rows(); y++) {
			int* row = out[y];
			for (int x = 0; x < in.cols(); x++) {
				row[x] = in.entries(i++);
			}
		}
	}

	void PackPayoffs(const DPPayoffs& payoffs,
									 proto::PayoffFeature& data,
									 const string& description) {
		PackMatrix(payoffs.wall_scores[0], *data.mutable_left());
		PackMatrix(payoffs.wall_scores[1], *data.mutable_right());
		if (!description.empty()) {
			data.set_description(description);
		}
	}

	void PackPayoffs(const MatF& payoffs,
									 proto::PayoffFeature& data,
									 const string& description) {
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

	// PackFeatures moved to progs/compute_payoff_features.cpp to remove
	// dependency on JointPayoffGen

	void UnpackFeatures(const proto::FrameWithFeatures& data,
											PayoffFeatures& feature_stack) {
		feature_stack.features.resize(data.features_size());
		feature_stack.descriptions.resize(data.features_size());
		for (int i = 0; i < data.features_size(); i++) {
			UnpackPayoffs(data.features(i), feature_stack.features[i]);
			if (data.features(i).has_description()) {
				feature_stack.descriptions[i] = data.features(i).description();
			}
		}
	}

	void CompilePayoffs(const PayoffFeatures& features,
											const ManhattanHyperParameters& params,
											DPPayoffs& payoffs) {
		// Copy penalties
		payoffs.wall_penalty = params.corner_penalty;
		payoffs.occl_penalty = params.occlusion_penalty;

		// Mix payoffs
		CHECK(!features.features.empty());
		CHECK_EQ(params.weights.Size(), features.features.size());
		payoffs.Resize(matrix_size(features.features[0]));
		for (int i = 0; i < params.weights.size(); i++) {
			payoffs.Add(features.features[i], params.weights[i]);
		}
	}
}
