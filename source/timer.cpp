#include "timer.h"

#include <sys/time.h>
#include <iostream>

#include <boost/format.hpp>

#include "common_types.h"

namespace indoor_context {
	static const long US_PER_MS = 1000;
	static const long US_PER_SEC = US_PER_MS * 1000;
	static const long US_PER_MIN = 60 * US_PER_SEC;

	Timer::Timer() {
		gettimeofday(&start, NULL);
	}

	Timer::Timer(const string& s) : str(s) {
		gettimeofday(&start, NULL);
	}

	ostream& operator<<(ostream& o, const Timer& t) {	
		struct timeval end;
		gettimeofday(&end, NULL);
		long dt = (end.tv_usec - t.start.tv_usec) + (end.tv_sec - t.start.tv_sec) * US_PER_SEC;

		o << t.str << ": ";
		if (dt > US_PER_MIN) {
			long mins = dt / US_PER_MIN;
			long secs = (dt % US_PER_MIN) / US_PER_SEC;
			o << boost::format("%ldm%lds") % mins % secs << endl;
		} else if (dt > US_PER_SEC) {
			float secs = 1. * dt / US_PER_SEC;
			o << boost::format("%.3fs") % secs << endl;
		} else if (dt > US_PER_MS) {
			float millis = 1. * dt / US_PER_MS;
			o << boost::format("%.3fms") % millis << endl;
		} else {
			o << boost::format("%ldus") % dt << endl;
		}

		return o;
	}


	ScopedTimer::ScopedTimer() {
	}

	ScopedTimer::ScopedTimer(const char* s) : t(s) {
	}

	ScopedTimer::ScopedTimer(const string& s) : t(s) {
	}

	ScopedTimer::~ScopedTimer() {
		DLOG << t;
	}	
}
