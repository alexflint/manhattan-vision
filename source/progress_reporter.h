#pragma once

#include <time.h>
#include <boost/thread.hpp>
#include "common_types.h"

namespace indoor_context {

	// Report progress on the commandline with automatic selection of
	// progress interval. Is thread safe.
	class ProgressReporter {
		int n;
		int i;
		int report_interval;
		int next_report;
		clock_t start_clock;  // value of clock() at construction()
		boost::mutex mtx;
	public:
		// Initialize a progress reporter with NN total steps
		ProgressReporter(int nn, string title = "");
		// Increment the current progress by 1
		void Increment();
	private:
		// these are private because making them public would cause thread
		// safety issues...

		// Update the current progress, 0 <= v <= NN
		void Update(int v);
		// Report timing information
		void ReportTiming() const;
	};
}
