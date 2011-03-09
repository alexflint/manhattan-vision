#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "monocular_payoffs.h"
#include "stereo_payoffs.h"
#include "point_cloud_payoffs.h"
#include "line_sweep_features.h"
#include "manhattan_dp.h"

namespace indoor_context {

	class JointPayoffGen {
	public:
		// These are initialized at construction from various gvars. You can
		// modify them at any point before calling Compute().
		double mono_weight; // weight for monocular payoffs
		double occlusion_weight;  // weight for occlusion payoffs computed from point cloud
		double agreement_weight;  // weight for agreement payoffs computed from point cloud
		double stereo_weight;  // total weight for stereo payoffs (shared between auxiliary frames)

		// The payoff generators
		LineSweepObjectiveGen objective_gen;  // for monocular payoffs
		MonocularPayoffGen mono_gen;
		PointCloudPayoffs point_cloud_gen;
		ptr_vector<StereoPayoffGen> stereo_gens;  // one for each auxiliary frame

		// Pointers to inputs
		const PosedImage* input;
		DPGeometryWithScale geometry;

		// The joint payoffs (output)
		DPPayoffs payoffs;

		// Reads gvars
		JointPayoffGen();
		// Reads gvars and computes
		JointPayoffGen(const PosedImage& image,
									 const DPGeometryWithScale& geometry,
									 const vector<Vec3>& point_cloud,
									 const vector<const PosedImage*>& aux_images);

		// Reset weights from gvar values
		void RevertWeights();

		// Compute joint payoffs
		void Compute(const PosedImage& image,
								 const DPGeometryWithScale& geometry,
								 const vector<Vec3>& point_cloud,
								 const vector<const PosedImage*>& aux_images);

		// Variant of above with non-const aux images, for convenience
		void Compute(const PosedImage& image,
								 const DPGeometryWithScale& geometry,
								 const vector<Vec3>& point_cloud,
								 const vector<PosedImage*>& aux_images);
	};
}
