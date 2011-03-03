#pragma once

#include "common_types.h"

namespace indoor_context {
	// Base class for training cases
	struct SVMLightCase {
		VecF feature;
		int label;
	};

	void WriteSVMProblem(const vector<SVMLightCase*>& cases,
											 const string& problem_name);
	void WriteSVMProblem(const vector<VecF*>& features,
											 const vector<int>& labels,
											 const string& problem_name);


	void TrainSVM(const string& problem_name,
								float cost_factor=-1,  // defaults to a gvar
								int verbosity=-1);  // defaults to a gvar
	void EvaluateSVM(const string& problem_name,
									 const string& model_name,
									 int verbosity=-1);
	void ReadSVMResponses(const string& problem_name,
												vector<float>& responses);


	void TrainMultiClassSVM(const string& problem_name,
													double margin_cost=-1,  // defaults to a gvar
													int verbosity=-1);  // defaults to a gvar
	void EvaluateMultiClassSVM(const string& problem_name,
														 const string& model_name,
														 int verbosity=-1);
	void ReadMultiClassSVMResponses(const string& problem_name,
																	vector<int>& classes,
																	MatF& responses);
}
