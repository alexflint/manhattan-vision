#pragma once

#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "manhattan_dp.h"
#include "payoffs.pb.h"

namespace indoor_context {
	class JointPayoffGen;

	// Represents mixing coefficients and complexity penalties for
	// manhattan reconstruction.
	class ManhattanHyperParameters {
	public:
		VecF weights;
		float corner_penalty;
		float occlusion_penalty;
	};

	// Represents monocular, stereo, and 3D payoffs for a particular image.
	// Currently un-used.
	class PayoffFeatures {
	public:
		ptr_vector<DPPayoffs> features;
		vector<string> descriptions;
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
	void PackMatrix(const MatI& in, proto::MatI& out);
	// Unpack a VNL matrix from a protocol buffer
	void UnpackMatrix(const proto::MatF& in, MatF& out);
	// Unpack a VNL matrix from a protocol buffer
	void UnpackMatrix(const proto::MatI& in, MatI& out);

	// Pack payoff features into a protocol buffer
	void PackFeatures(const JointPayoffGen& joint,
										proto::FrameWithFeatures& data);
	// Unpack payoff features from a protocol buffer
	void UnpackFeatures(const proto::FrameWithFeatures& data,
											PayoffFeatures& features);
	// Compile a payoff function from features and mixing parameters. If
	// not null, the last parameter will be filled with derivatives of
	// the payoff function with respect to each parameter.
	void CompilePayoffs(const PayoffFeatures& features,
											const ManhattanHyperParameters& params,
											DPPayoffs& payoffs);
}
