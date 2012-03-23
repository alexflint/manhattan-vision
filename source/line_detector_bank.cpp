#include "line_detector_bank.h"

#include "common_types.h"
#include "vanishing_point_model.h"
#include "line_detector.h"
#include "map.h"

#include "range_utils.tpp"

namespace indoor_context {

	void LineDetectorBank::Configure(const Map& map) {
		map_ = &map;
		detection_sets.resize(map.frames.size());
	}

	void LineDetectorBank::ProcessFrame(int frame_id) {
		CHECK(!detection_sets.empty())
			<< "Must call LineDetectorBank::Configure() before Compute()";
		CHECK_INDEX(frame_id, detection_sets);
		if (detection_sets[frame_id].empty()) {
			const Frame& frame = *map_->GetFrameById(frame_id);
			CHECK(frame.image.loaded()) << "Image must be loaded by caller";
			line_detector.Compute(frame.image);
			copy_all_into(line_detector.detections, detection_sets[frame_id]);
		}
	}

	void LineDetectorBank::GetDetectionsFor(int frame_id,
																					vector<LineDetection>& detections) {
		ProcessFrame(frame_id);
		copy_all_into(detection_sets[frame_id], detections);
	}

	void LineDetectorBank::GetDetectionsFor(int frame_id,
																					vector<LineSegment>& segments) {
		ProcessFrame(frame_id);
		BOOST_FOREACH(const LineDetection& det, detection_sets[frame_id]) {
			segments.push_back(det.seg);
		}
	}

	void LineDetectorBank::GetObservationsFor(int frame_id,
																						vector<LineObservation>& obs) {
		ProcessFrame(frame_id);
		const Frame* frame = map_->GetFrameByIdOrDie(frame_id);
		BOOST_FOREACH(const LineDetection& det, detection_sets[frame_id]) {
			obs.push_back(LineObservation(&frame->image.pc(), det.seg));
		}
	}
}
