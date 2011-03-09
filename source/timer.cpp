#include <sys/time.h>
#include <boost/format.hpp>
#include "timer.h"
#include "common_types.h"

namespace indoor_context {
	static const long US_PER_MS = 1000;
	static const long US_PER_SEC = US_PER_MS * 1000;
	static const long US_PER_MIN = 60 * US_PER_SEC;

	scoped_timer::scoped_timer() {
		gettimeofday(&start, NULL);
	}

	scoped_timer::scoped_timer(const char* s) : str(s) {
		gettimeofday(&start, NULL);
	}

	scoped_timer::scoped_timer(const string& s) : str(s) {
		gettimeofday(&start, NULL);
	}

	scoped_timer::~scoped_timer() {
		struct timeval end;
		gettimeofday(&end, NULL);
		long dt = (end.tv_usec - start.tv_usec) + (end.tv_sec - start.tv_sec) * US_PER_SEC;

		DLOG_N << str << ": ";
		if (dt > US_PER_MIN) {
			long mins = dt / US_PER_MIN;
			long secs = (dt % US_PER_MIN) / US_PER_SEC;
			DLOG << boost::format("%ldm%lds") % mins % secs;
		} else if (dt > US_PER_SEC) {
			float secs = 1. * dt / US_PER_SEC;
			DLOG << boost::format("%.3fs") % secs;
		} else if (dt > US_PER_MS) {
			float millis = 1. * dt / US_PER_MS;
			DLOG << boost::format("%.3fms") % millis;
		} else {
			DLOG << boost::format("%ldus") % dt;
		}
	}
}
