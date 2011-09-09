#include "timer.h"

#include <sys/time.h>
#include <iostream>

#include <boost/format.hpp>

#include "common_types.h"

namespace indoor_context {
	static const long US_PER_MS = 1000;
	static const long US_PER_SEC = 1000 * US_PER_MS;
	static const long US_PER_MIN = 60 * US_PER_SEC;

	Timer::Timer() {
		gettimeofday(&start, NULL);
	}

	ostream& operator<<(ostream& o, const Timer& t) {	
		struct timeval end;
		gettimeofday(&end, NULL);
		long dt = (end.tv_usec - t.start.tv_usec) + (end.tv_sec - t.start.tv_sec) * US_PER_SEC;

		if (dt > US_PER_MIN) {
			long mins = dt / US_PER_MIN;
			long secs = (dt % US_PER_MIN) / US_PER_SEC;
			return o << boost::format("%ldm%lds") % mins % secs;
		} else if (dt > US_PER_SEC) {
			float secs = 1. * dt / US_PER_SEC;
			return o << boost::format("%.3fs") % secs;
		} else if (dt > US_PER_MS) {
			float millis = 1. * dt / US_PER_MS;
			return o << boost::format("%.3fms") % millis;
		} else {
			return o << boost::format("%ldus") % dt;
		}
	}

	ostream& operator<<(ostream& o, const ScopedTimer& t) {	
		return o << t.title << ": " << t.timer << endl;
	}

	ScopedTimer::ScopedTimer() {
	}

	ScopedTimer::ScopedTimer(const char* s) : title(s) {
	}

	ScopedTimer::ScopedTimer(const string& s) : title(s) {
	}

	ScopedTimer::~ScopedTimer() {
		DLOG << (*this);
	}	
}
