// Testing interop between numpy and VNL,TooN

#include <boost/ptr_container/ptr_vector.hpp>

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/numeric.hpp>

#include <numpy/noprefix.h>

#include "entrypoint_types.h"
#include "payoff_helpers.h"
#include "building_features.h"

#include "pywrappers/numpy_conversions.h"
#include "pywrappers/numpy_helpers.h"
#include "pywrappers/training.h"

using namespace toon;
using namespace boost::python;
namespace bp=boost::python;

using boost::ptr_vector;

template <typename Range>
struct SequenceFromPythonCopyConverter {
	static void* convertible(PyObject* obj) {
		return PySequence_Check(obj) ? obj : NULL;
	}

	static void construct(PyObject* obj,
												converter::rvalue_from_python_stage1_data* data) {
		typedef typename range_value<Range>::type T;
		typedef converter::rvalue_from_python_storage<Range> Storage;
		int len = PySequence_Length(obj);
		Range* storage = (Range*)((Storage*)data)->storage.bytes;
		new (storage) Range(len);   // TODO: get out the allocator, use it
		for (int i = 0; i < len; i++) {
			(*storage)[i] = extract<T>(PySequence_GetItem(obj, i));
		}
		data->convertible = storage;
	}
};

template <typename Sequence>
void RegisterSequenceConverter() {
	converter::registry::push_back(&SequenceFromPythonCopyConverter<Sequence>::convertible,
																 &SequenceFromPythonCopyConverter<Sequence>::construct,
																 type_id<Sequence>());
}

MatF GetWallScores(const DPPayoffs& payoffs, int i) {
	return payoffs.wall_scores[i];
}

VecI GetPathYs(const ManhattanHypothesis& soln) {
	return soln.path_ys;
}

VecI GetPathOrients(const ManhattanHypothesis& soln) {
	return soln.path_axes;
}

string GetSequenceName(const TrainingInstance& inst) {
	return inst.sequence;
}

string GetImagePath(const TrainingInstance& inst) {
	return inst.frame->image_file;
}

int GetFrameId(const TrainingInstance& inst) {
	return inst.frame->id;
}

VecF GetWeights(const ManhattanHyperParameters& params) {
	return params.weights;
}

float GetCornerPenalty(const ManhattanHyperParameters& params) {
	return params.corner_penalty;
}

float GetOcclusionPenalty(const ManhattanHyperParameters& params) {
	return params.occlusion_penalty;
}

const TrainingInstance& GetInstance(const ManhattanHypothesis& hyp) {
	CHECK_NOT_NULL(hyp.instance);
	return *hyp.instance;
}

double ComputeScore(const DPPayoffs& payoffs, const ManhattanHypothesis& hyp) {
	return payoffs.ComputeScore(hyp.path_ys,
															hyp.path_axes,
															hyp.num_corners,
															hyp.num_occlusions);
}

void ComputePhotometricFeaturesForInstance(PhotometricFeatures& ftrgen,
																					 const TrainingInstance& inst) {
	ftrgen.Compute(inst.frame->image);
}

int NumFeatures(const PhotometricFeatures& ftrgen) {
	return ftrgen.features.size();
}

MatF GetFeature(const PhotometricFeatures& ftrgen, int i) {
	CHECK_INDEX(i, ftrgen.features);
	return *ftrgen.features[i];
}

string GetFeatureName(const PhotometricFeatures& ftrgen, int i) {
	CHECK_INDEX(i, ftrgen.feature_strings);
	return ftrgen.feature_strings[i];
}

BOOST_PYTHON_MODULE(py_indoor_context) {
	AssertionManager::SetExceptionMode();
	InitVars();
	InitializeNumpy();
	RegisterNumpyConversions();
	RegisterSequenceConverter<vector<int> >();
	RegisterSequenceConverter<vector<float> >();
	RegisterSequenceConverter<vector<double> >();

	// TrainingInstance
	class_<TrainingInstance, boost::noncopyable>("TrainingInstance")
		.def("GetSequenceName", &GetSequenceName)
		.def("GetFrameId", &GetFrameId)
		.def("GetImagePath", &GetImagePath)

		.def("ConfigureL1Loss", &TrainingInstance::ConfigureL1Loss)
		.def("ConfigureLabellingLoss", &TrainingInstance::ConfigureLabellingLoss)
		.def("ConfigureDepthLoss", &TrainingInstance::ConfigureDepthLoss)

		.def("GetGroundTruth", &TrainingInstance::GetGroundTruth)
		.def("GetGroundTruthLabels", &TrainingInstance::GetGroundTruthLabels)
		.def("GetGroundTruthDepths", &TrainingInstance::GetGroundTruthDepths)
		.def("OutputGroundTruthViz", &TrainingInstance::OutputGroundTruthViz)

		.def("GetLossTerms", &TrainingInstance::GetLossTerms)
		.def("ComputeLoss", &TrainingInstance::ComputeLoss)
		.def("ComputeGridLabels", &TrainingInstance::ComputeLabels)
		.def("ComputeGridDepths", &TrainingInstance::ComputeDepths)
		;
	
	// TrainingManager
	class_<TrainingManager, boost::noncopyable>("TrainingManager")
		.def("LoadSequence", &TrainingManager::LoadSequence)
		.def("GetInstance",
				 &TrainingManager::GetInstance,
				 return_internal_reference<>())
		.def("NumInstances", &TrainingManager::NumInstances)
		.def("NormalizeFeatures", &TrainingManager::NormalizeFeatures)
		;

	// Manhattan Inference
	class_<ManhattanInference, boost::noncopyable>("ManhattanInference")
		.def("Solve", &ManhattanInference::Solve)
		.def("GetSolutionScore", &ManhattanInference::GetSolutionScore)
		.def("GetSolutionLabels", &ManhattanInference::GetSolutionLabels)
		.def("GetSolutionDepths", &ManhattanInference::GetSolutionDepths)
		.def("OutputSolutionViz", &ManhattanInference::OutputSolutionViz)
		;

	// ManhattanHyperParameters
	class_<ManhattanHyperParameters>("ManhattanHyperParameters",
																	 init<const VecF&,float,float>())
		.def("GetWeights", &GetWeights)
		.def("GetCornerPenalty", &GetCornerPenalty)
		.def("GetOcclusionPenalty", &GetOcclusionPenalty)
		;

	// DPPayoffs
	class_<DPPayoffs, boost::noncopyable>("DPPayoffs")
		.def("Get", &GetWallScores)
		.def("ComputeScore", &ComputeScore)
		;

	// ManhattanHypothesis
	class_<ManhattanHypothesis>("ManhattanHypothesis")
		.def("GetPath", &GetPathYs)
		.def("GetOrients", &GetPathOrients)
		.def("GetInstance", &GetInstance, return_internal_reference<>())
		;

	// FeatureManager
	class_<FeatureManager, boost::noncopyable>("FeatureManager",
																						 init<const string&>())
		.def("ComputeMultiViewFeatures", &FeatureManager::ComputeMultiViewFeatures)
		.def("ComputeSweepFeatures", &FeatureManager::ComputeSweepFeatures)
		.def("ComputeMonoFeatures", &FeatureManager::ComputeMonoFeatures)
		.def("ComputeMockFeatures", &FeatureManager::ComputeMockFeatures)

		.def("CommitFeatures", &FeatureManager::CommitFeatures)
		.def("LoadFeaturesFor", &FeatureManager::LoadFeaturesFor)

		.def("NumFeatures", &FeatureManager::NumFeatures)
		.def("GetFeature", &FeatureManager::GetFeature)
		.def("GetFeatureComment",
				 &FeatureManager::GetFeatureComment,
				 return_value_policy<return_by_value>())

		.def("Compile", &FeatureManager::Compile)
		.def("CompileWithLoss", &FeatureManager::CompileWithLoss)
		.def("ComputeFeatureForHypothesis",
				 &FeatureManager::ComputeFeatureForHypothesis)
		;

		/*class_<PhotometricFeatures, boost::noncopyable>("PhotometricFeatures")
		.def("Configure", &PhotometricFeatures::Configure)
		.def("IsActive", &PhotometricFeatures::IsActive)
		.def("Compute", &ComputePhotometricFeaturesForInstance)
		.def("NumFeatures", &NumFeatures)
		.def("GetFeature", &GetFeature)
		.def("GetFeatureName", &GetFeatureName)
		;*/
}
