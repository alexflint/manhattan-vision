#pragma once

#include <sys/time.h>
#include <string>
#include <iostream>

// This macro is used like this:
//
// code: TIMED("my function") foo()
// output: my function time: XX.XXms
//
// code:
// TIMED("some block") {
//   foo();
//   bar();
//   baz();
// }
// output:
// some block: XX.XXms
//
// code:
// TIMED("the loop") for(int i = 0; i < n; i++) {
//   ...
// }
// output:
// the loop: XX.XXms

#define TIMED(str) if (indoor_context::ScopedTimer __t = str)
#define TIMED_SECTION(str) TITLED(str) if (indoor_context::ScopedTimer __t = "Time taken")

namespace indoor_context {
	// Starts a timer on construction.
	struct Timer {
		struct timeval start;
		Timer();
	};

	// Starts a timer on construction and reports its value on destruction
	struct ScopedTimer {
		Timer timer;
		std::string title;
		ScopedTimer();
		ScopedTimer(const char* title);
		ScopedTimer(const std::string& title);
		~ScopedTimer();
		inline operator bool() { return true; }
	};

	// Stream insertion for timer
	std::ostream& operator<<(std::ostream& o, const Timer& t);
	// Stream insertion for scoped timer
	std::ostream& operator<<(std::ostream& o, const ScopedTimer& t);
}
