#pragma once

#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "vanishing_point_model.h"
#include "line_detector.h"

namespace indoor_context {
	class Map;

	// Manages lazy line detection across multiple frames
	class LineDetectorBank {
	public:
		boost::ptr_vector<vector<LineDetection> > detection_sets;
		const Map* map_;

		// these are here for efficiency only
		CannyLineDetector line_detector;

		LineDetectorBank() { }
		LineDetectorBank(const Map& map) {
			Configure(map);
		}

		// Detect line segments in each line
		void Configure(const Map& map);

		// Detect lines in a frame
		void ProcessFrame(int frame_id);

		// Compute rotation from all line detections in all frames
		void GetDetectionsFor(int frame_id,
													vector<LineDetection>& detections);
		void GetDetectionsFor(int frame_id,
													vector<LineSegment>& segments);
		void GetObservationsFor(int frame_id,
														vector<LineObservation>& observations);
	};
}
