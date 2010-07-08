#include <time.h>
#include <iostream>
#include <iomanip>

#include "progress_reporter.h"

#include "common_types.h"
#include "log.h"

namespace indoor_context {

	const int kValidIntervals[] = {1, 2, 5, 10, 20, 50};
	const float kMinReportTime = 2;  // in seconds

	ProgressReporter::ProgressReporter(int nn, string title)
		: n(nn),
			i(0),
			report_interval(-1),
			next_report(-1),
			start_clock(clock()) {
		if (title.size() > 0) {
			DLOG << title << endl;
		}
	}

	void ProgressReporter::Update(int v) {
		i = v;
		const int progress = i*100/n;
		const double elapsed = 1.0*(clock() - start_clock) / CLOCKS_PER_SEC;
		if (report_interval == -1 && elapsed >= kMinReportTime) {
			for (int j = 0; j < 6; j++) {
				if (kValidIntervals[j] < progress) {
					report_interval = kValidIntervals[j];
					next_report = kValidIntervals[j];
				}
			}
		}
		while (next_report != -1 && progress >= next_report) {
			DLOG << next_report << "%" << endl;
			next_report += report_interval;
		}
	}

	void ProgressReporter::Increment() {
		// Don't worry about inefficiency of locking a mutex every time
		// because we shouldn't be using a ProgressReporter at all if
		// efficiency is a major issue!
		mutex::scoped_lock lock(mtx);
		Update(i+1);
	}

	void ProgressReporter::ReportTiming() const {
		double secs = 1.0*(clock() - start_clock) / CLOCKS_PER_SEC;
		DLOG << "Processed " << n << " items in "
				 << fixed << setprecision(1) << secs << " seconds\n";
		DLOG << "Average time per item: " << fixed << setprecision(3)
				 << (secs / n) << endl;
		DLOG << "Average items per second: " << fixed << setprecision(2)
				 << (1.0*n / secs) << endl;
	}
}
