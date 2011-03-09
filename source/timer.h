#pragma once

#include <time.h>
#include <string>

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

#define TIMED(str) if (scoped_timer __t = str)
#define TIMED_SECTION(str) TITLED(str) if (scoped_timer __t = "Time taken")

namespace indoor_context {
	// Starts a timer on construction and reports its value on destruction
	struct scoped_timer {
		const std::string str;
		struct timeval start;
		scoped_timer();
		scoped_timer(const char* s);
		scoped_timer(const std::string& s);
		~scoped_timer();
		inline operator bool() { return true; }
	};
}
