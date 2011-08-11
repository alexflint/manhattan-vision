#include "svm_light_wrappers.h"

#include <stdio.h>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include "common_types.h"
#include "timer.h"

namespace indoor_context {
	using boost::format;
	using boost::str;

	lazyvar<string> gvScratchDir("SVMLight.ScratchDir");
	lazyvar<int> gvDefaultVerbosity("SVMLight.DefaultVerbosity");
	lazyvar<float> gvDefaultMarginPenalty("SVMLight.DefaultMarginPenalty");
	lazyvar<int> gvPrintCommands("SVMLight.PrintCommands");

	lazyvar<string> gvBaseDir("SVMLight.BaseDir");
	lazyvar<string> gvTrainCmd("SVMLight.TrainCmd");
	lazyvar<string> gvClassifyCmd("SVMLight.ClassifyCmd");

	lazyvar<string> gvMultiClassBaseDir("SVMLight.MultiClass.BaseDir");
	lazyvar<string> gvMultiClassTrainCmd("SVMLight.MultiClass.TrainCmd");
	lazyvar<string> gvMultiClassClassifyCmd("SVMLight.MultiClass.ClassifyCmd");

	// Run an external command and redirect it through DLOG
	int logged_system(const string& command) {
		if (*gvPrintCommands) {
			DLOG << "Executing: " << command;
		}

		// Open stream to the external process
		FILE* fd = popen(command.c_str(), "r");
		if (!fd) {
			perror("popen");
			exit(-1);
		}

		// Read and pass through LogStream
		char buf[1024];
		const char* p;
		do {
			if (p = fgets(buf, 1024, fd)) {
				LogManager::GetLogStream() << p;			// Use '<<' rather than s.write() since we need to look for '\0'
			}
		} while (p != NULL && !feof(fd));

		// Close the stream and check the return value
		int ret = pclose(fd);  // a return value of -1 indicates an error
													 // in pclose, otherwise ret is the child
													 // process return value
		// End the line if not already terminated
		LogManager::EndCurrentLine();

		// Check return value
		if (ret == -1) {
			perror("pclose");
			exit(-1);
		}
		return ret;
	}

	string File(const string& name, const string& ext) {
		CHECK_GT(name.size(), 0);
		CHECK_GT(ext.size(), 0);
		static const fs::path scratch(*gvScratchDir);
		return (scratch / (name+"."+ext)).string();
	}

	string ProblemFile(const string& name) {
		return File(name, "problem");
	}

	string ModelFile(const string& name) {
		return File(name, "model");
	}

	string ResponseFile(const string& name) {
		return File(name, "out");
	}

	void TrainSVM(const string& prob,
								const float cost_factor,
								int verbosity) {
		//if (margin_penalty < 0) margin_penalty = *gvDefaultMarginPenalty;
		if (verbosity == -1) verbosity = *gvDefaultVerbosity;
		string train_cmd = str(format("%s/%s -v %d -j %f %s %s")
													 % *gvBaseDir
													 % *gvTrainCmd
													 % verbosity
													 % cost_factor
													 % *gvDefaultMarginPenalty
													 % ProblemFile(prob)
													 % ModelFile(prob));
		int ret = logged_system(train_cmd);
		CHECK_EQ(ret, 0);
	}

	void TrainMultiClassSVM(const string& prob,
													double margin_penalty,
													int verbosity) {
		if (margin_penalty < 0) margin_penalty = *gvDefaultMarginPenalty;
		if (verbosity == -1) verbosity = *gvDefaultVerbosity;
		string train_cmd = str(format("%s/%s -v %d -c %f %s %s")
													 % *gvMultiClassBaseDir
													 % *gvMultiClassTrainCmd
													 % verbosity
													 % margin_penalty
													 % ProblemFile(prob)
													 % ModelFile(prob));
		int ret = logged_system(train_cmd);
		CHECK_EQ(ret, 0);
	}

	void EvaluateSVM(const string& prob, const string& model, int verbosity) {
		if (verbosity == -1) verbosity = *gvDefaultVerbosity;
		string eval_cmd = str(format("%s/%s -v %d %s %s %s")
													% *gvBaseDir
													% *gvClassifyCmd
													% verbosity
													% ProblemFile(prob)
													% ModelFile(model)
													% ResponseFile(prob));
		int ret = logged_system(eval_cmd);
		CHECK_EQ(ret, 0);
	}


	void EvaluateMultiClassSVM(const string& prob, const string& model, int verbosity) {
		if (verbosity == -1) verbosity = *gvDefaultVerbosity;
		string eval_cmd = str(format("%s/%s -v %d %s %s %s")
													% *gvMultiClassBaseDir
													% *gvMultiClassClassifyCmd
													% verbosity
													% ProblemFile(prob)
													% ModelFile(model)
													% ResponseFile(prob));
		int ret = logged_system(eval_cmd);
		CHECK_EQ(ret, 0);
	}

	void WriteSVMProblem(const vector<SVMLightCase*>& cases, const string& prob) {
		string file = ProblemFile(prob);
		FILE* fd = fopen(file.c_str(), "w+");
		CHECK(fd) << "Failed to open file: " << file;
		BOOST_FOREACH(const SVMLightCase* c, cases) {
			fprintf(fd, "%d", c->label);
			for (int i = 0; i < c->feature.Size(); i++) {
				fprintf(fd, " %d:%f", i+1, c->feature[i]);
			}
			fputs("\n", fd);
		}
		CHECK(fclose(fd) == 0);
	}

	void WriteSVMProblem(const vector<VecF*>& features,
											 const vector<int>& labels,
											 const string& prob) {
		string file = ProblemFile(prob);
		FILE* fd = fopen(file.c_str(), "w+");
		CHECK(fd) << "Failed to open file: " << file;
		CHECK_EQ(labels.size(), features.size());
		for (int i = 0; i < features.size(); i++) {
			fprintf(fd, "%d", labels[i]);
			const VecF& ftr = *features[i];
			for (int j = 0; j < ftr.size(); j++) {
				fprintf(fd, " %d:%f", j+1, ftr[j]);
			}
			fputs("\n", fd);
		}
		CHECK(fclose(fd) == 0);
	}

	void ReadSVMResponses(const string& prob, vector<float>& responses) {
		string file = ResponseFile(prob);
		CHECK_GT(responses.size(), 0) << "responses.size() should be same size as the input file";
		FILE* results_fd = fopen(file.c_str(), "r");
		CHECK_NOT_NULL(results_fd) << "Failed to open "<<file;
		for (int i = 0; i < responses.size(); i++) {
			int r = fscanf(results_fd, "%f", &responses[i]);
			CHECK(r != EOF) << "Failed to read classifier output for case "
											<< i << " (of " << responses.size() << ") from " << file;
		}
	}

	void ReadMultiClassSVMResponses(const string& prob,
																	vector<int>& classes,
																	MatF& responses) {
		string file = ResponseFile(prob);
		CHECK_GT(classes.size(), 0) << "classes.size() should be same size as the input file";
		CHECK_EQ(responses.Rows(), classes.size());
		CHECK(responses.Rows() > 0 && responses.Cols() > 0)
			<< "Size of responses should be NUM_EXAMPLES x NUM_CLASSES";

		FILE* results_fd = fopen(file.c_str(), "r");
		CHECK_NOT_NULL(results_fd) << "Failed to open "<<file;

		for (int i = 0; i < responses.Rows(); i++) {
			int r = fscanf(results_fd, "%d", &classes[i]);
			CHECK(r != EOF) << "Failed to read classifier output for case "
											<< i << " (of " << classes.size() << ") from " << file;
			for (int j = 0; j < responses.Cols(); j++) {
				int r = fscanf(results_fd, "%f", &responses[i][j]);
				CHECK(r != EOF) << "Failed to read classifier output for case "
												<< i << " (of " << classes.size() << ") from " << file;
			}
		}
	}
}
