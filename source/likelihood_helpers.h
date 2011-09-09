#include "common_types.h"

namespace indoor_context {
	class ModelLikelihood;
	class FeatureLikelihood;
	class ManhattanHyperParameters;
	namespace proto { class PayoffFeatureSet; }

	double EvaluateModelLogLik(ModelLikelihood& likelihood,
														 const proto::PayoffFeatureSet& featureset,
														 const Vec2& lambda);

	double EvaluateFeatureLogLik(FeatureLikelihood& likelihood,
															 const proto::PayoffFeatureSet& featureset,
															 const Vector<>& theta);

	double EvaluateLikelihood(const ManhattanHyperParameters& params,
														const string& features_file,
														bool logit_likelihood=false);
}
