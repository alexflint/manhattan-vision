#include <boost/ptr_container/ptr_vector.hpp>

#include "entrypoint_types.h"
#include "map.h"
#include "map.pb.h"

#include "payoff_helpers.h"
#include "manhattan_dp.h"
#include "manhattan_ground_truth.h"

namespace indoor_context {

	class TrainingInstance {
	public:
		Frame* frame;
		DPGeometryWithScale geometry;
		PayoffFeatures features;
		ManhattanGroundTruth gt;

		MatF loss_terms;

		void Configure(const Frame& frame);

		double ComputeLoss(const DPSolution& hypothesis) const;
		void ComputeLossTerms() const;
	};

	void TrainingInstance::Configure(const Frame& f, double zfloor, double zceil) {
		frame = &f;
		geometry.Configure(f.image.pc(), zfloor, zceil);
	}

	double TrainingInstance::ComputeLoss(const DPSolution& hyp) const {
		CHECK(false) << "Not implemented";
	}

	void TrainingInstance::ComputeLossTerms() const {
		CHECK(false) << "Not implemented";
	}



	class TrainingManager {
	public:
		Map map;
		proto::TruthedMap gt_map;
		ptr_vector<TrainingInstance> instances;

		ManhattanDP solver;

		void LoadSequence(const string& sequence,
											vector<int> frame_ids);

		// Return values below will be invalidated by the next call to
		// either of the below
		const DPSolution& Solve(const DPPayoffs& payoffs) const;
		const DPSolution& SolveLossAugmented(const DPPayoffs& payoffs) const;
	};

	const DPSolution& TrainingManager::Solve(const TrainingInstance& instance,
																					 const DPPayoffs& payoffs) const {
		solver.Compute(instance.geometry, payoffs);
		return solver.solution;
	}

	const DPSolution& TrainingInstance::SolveLossAugmented(const DPPayoffs& payoffs) const {
		CHECK(false) << "Not implemented";
	}
}
