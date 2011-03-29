#include "payoff_helpers.h"

#include <boost/filesystem/operations.hpp>

#include "common_types.h"
#include "joint_payoffs.h"
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

	void PackFeatures(const vector<int>& aux_ids,
										const JointPayoffGen& joint,
										proto::PayoffFeatures& precomputed) {
		PackMatrix(joint.mono_gen.payoffs.wall_scores[0], *precomputed.mutable_mono0());
		PackMatrix(joint.mono_gen.payoffs.wall_scores[1], *precomputed.mutable_mono1());

		PackMatrix(joint.point_cloud_gen.agreement_payoffs, *precomputed.mutable_agreement());
		PackMatrix(joint.point_cloud_gen.occlusion_payoffs, *precomputed.mutable_occlusion());
		precomputed.set_agreement_sigma(joint.point_cloud_gen.agreement_sigma);

		precomputed.clear_stereo_aux_ids();
		precomputed.clear_stereo();
		for (int i = 0; i < aux_ids.size(); i++) {
			precomputed.add_stereo_aux_ids(aux_ids[i]);
			PackMatrix(joint.stereo_gens[i].payoffs, *precomputed.add_stereo());
		}
	}

	void UnpackFeatures(const proto::PayoffFeatures& precomputed,
											PayoffFeatures& features) {
		UnpackMatrix(precomputed.mono0(), features.mono_payoffs.wall_scores[0]);
		UnpackMatrix(precomputed.mono1(), features.mono_payoffs.wall_scores[1]);
		UnpackMatrix(precomputed.agreement(), features.agreement_payoffs);
		UnpackMatrix(precomputed.occlusion(), features.occlusion_payoffs);

		features.stereo_payoffs.clear();
		CHECK_EQ(precomputed.stereo_aux_ids_size(), precomputed.stereo_size());
		for (int i = 0; i < precomputed.stereo_size(); i++) {
			features.stereo_payoffs.push_back(new MatF);
			UnpackMatrix(precomputed.stereo(i), features.stereo_payoffs.back());
		}
	}

	void LoadFeatures(const string& file, PayoffFeatures& features) {
		proto::PayoffFeatures data;
		ReadProto(file, data);
		UnpackFeatures(data, features);
	}

	void SaveFeatures(const string& file,
										const vector<int>& aux_ids,
										const JointPayoffGen& joint) {
		proto::PayoffFeatures data;
		PackFeatures(aux_ids, joint, data);
		sofstream proto_out(file);
		data.SerializeToOstream(&proto_out);
	}

	void CompilePayoffs(const PayoffFeatures& features,
											const JointPayoffParameters& params,
											DPPayoffs& payoffs,
											ptr_vector<DPPayoffs>* J_payoffs) {
		// Copy penalties
		payoffs.wall_penalty = params.corner_penalty;
		payoffs.occl_penalty = params.occlusion_penalty;

		// Mix payoffs together
		payoffs.Resize(matrix_size(features.mono_payoffs));
		payoffs.Add(features.mono_payoffs, params.mono_weight);
		payoffs.Add(features.agreement_payoffs, params.agreement_weight);
		payoffs.Add(features.occlusion_payoffs, params.occlusion_weight);
		if (!features.stereo_payoffs.empty()) {
			double stereo_weight_per_aux = params.stereo_weight / features.stereo_payoffs.size();
			BOOST_FOREACH(const MatF& po, features.stereo_payoffs) {
				payoffs.Add(po, stereo_weight_per_aux);
			}
		}

		// Compute derivatives
		if (J_payoffs != NULL) {
			J_payoffs->resize(4);
			for (int i = 0; i < 4; i++) {
				J_payoffs->at(i).Resize(matrix_size(payoffs));
			}

			// The order here must match that in compute_loglik.cpp and compute_loglik.m
			J_payoffs->at(0).Add(features.mono_payoffs);
			BOOST_FOREACH(const MatF& po, features.stereo_payoffs) {
				J_payoffs->at(1).Add(po, 1. / features.stereo_payoffs.size());
			}
			J_payoffs->at(2).Add(features.agreement_payoffs);
			J_payoffs->at(3).Add(features.occlusion_payoffs);
		}
	}
}
