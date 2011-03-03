#pragma once

#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "manhattan_dp.h"
#include "image_bundle.h"
#include "svm_light_wrappers.h"

namespace indoor_context {

	struct PixelCase : public SVMLightCase {
		int frame_id;
		int frame_index;
		Vec2I location;
	};

	struct FrameCase {
		int frame_id;
		DPGeometry geometry;
		ptr_vector<PixelCase> pixels;
		ManhattanGroundTruth ground_truth;

		// The original image
		const PosedImage* image;
		// The SVM outputs for each pixel, for each class
		MatF responses[3];
		// The objective function computed from the above
		DPObjective obj;
	};

	/*void OutputLabelViz(const ImageRGB<byte>& bg,
											const MatI& clas,
											const string& file);

	void OutputResponseViz(const ImageRGB<byte>& bg,
												 const MatF& responses,
												 const string& file);*/

	/*
	class SVMLightWrapper {
	public:
		string basename;

		SVMLightWrapper();
		SVMLightWrapper(const string& basename);

		void Configure(const string& basename);

		void Train(const string& problem_file, const string& model_file);
		void Evaluate(const string& problem_file, const string& model_file);

	};

	SVMLightWrapper::SVMLightWrapper() {
	}

	SVMLightWrapper::SVMLightWrapper(const string& basename) {
		Configure(basename);
	}

	SVMLightWrapper::Configure(const string& basename) {
		base_name = name;
	}
	*/
}
