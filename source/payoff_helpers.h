#pragma once

#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "dp_payoffs.h"
#include "payoffs.pb.h"

namespace indoor_context {
	// Represents mixing coefficients and complexity penalties for
	// manhattan reconstruction.
	class ManhattanHyperParameters {
	public:
		VecF weights;
		float corner_penalty;
		float occlusion_penalty;
		ManhattanHyperParameters(const VecF& weights,
														 float corner_penalty,
														 float occlusion_penalty);
	};

	// Represents a set of features in payoff form
	class PayoffFeatures {
	public:
		boost::ptr_vector<DPPayoffs> features;
		vector<string> descriptions;
		// Clear all
		void Clear();
		// Append a copy of a payoff matrix
		void AddCopy(const DPPayoffs& payoffs, const string& desc);
		void AddCopy(const MatF& payoffs, const string& desc);
		// Compile a payoff function from features and weights
		void Compile(const ManhattanHyperParameters& params,
								 DPPayoffs& out_payoffs) const;
	};

	// Represents performance statistics for a reconstruction algorithm.
	class ManhattanPerformanceStatistics {
	public:
		ManhattanPerformanceStatistics()
			: num_frames(0), sum_depth_error(0), sum_labelling_error(0) { }
		int num_frames;
		float sum_depth_error;
		float sum_labelling_error;
	};

	// Pack a VNL matrix into a protocol buffer
	void PackMatrix(const MatF& in, proto::MatF& out);
	// Pack a VNL matrix into a protocol buffer
	void PackMatrix(const MatD& in, proto::MatF& out);
	// Pack a VNL matrix into a protocol buffer
	void PackMatrix(const MatI& in, proto::MatI& out);
	// Unpack a VNL matrix from a protocol buffer
	void UnpackMatrix(const proto::MatF& in, MatF& out);
	// Unpack a VNL matrix from a protocol buffer
	void UnpackMatrix(const proto::MatI& in, MatI& out);

	// Pack payoffs into a protocol buffer
	void PackPayoffs(const DPPayoffs& payoffs,
									 const string& description,
									 proto::PayoffFeature& data);
	void PackPayoffs(const MatF& payoffs,
									 const string& description,
									 proto::PayoffFeature& data);
	// Unpack payoffs from a protocol buffer
	void UnpackPayoffs(const proto::PayoffFeature& data,
										 DPPayoffs& payoffs);

	// Pack features into a protocol buffer
	void PackFeatures(const PayoffFeatures& fset,
										proto::PayoffFeatureSet& data);
	// Unpack payoff features from a protocol buffer
	void UnpackFeatures(const proto::PayoffFeatureSet& data,
											PayoffFeatures& features);

	// Write features to file
	void WriteFeatures(const string& path, const PayoffFeatures& fset);
	// Read features from file
	void ReadFeatures(const string& path, PayoffFeatures& fset);
}
